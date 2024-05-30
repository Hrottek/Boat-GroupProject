#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <EEPROM.h>

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

#define SONAR_MAX_DISTANCE 600

/// Display
#define TFT_CS 10
#define TFT_RST 8
#define TFT_DC 9

#define DISPLAY_BUTTON_MENU 4
#define DISPLAY_BUTTON_SELECT 3
#define DISPLAY_BUTTON_CONFIRM 2
#define DISPLAY_BUTTON_HOME 5

#define DISPLAY_WIDTH 320
#define DISPLAY_HEIGHT 240

#define DISPLAY_BUTTON_WIDTH 180
#define DISPLAY_BUTTON_HEIGHT 30

#define DISPLAY_TOP_BAR_COLOR ST77XX_BLACK
#define DISPLAY_MENU_BAR_COLOR ST77XX_BLACK
#define DISPLAY_MENU_BAR_BUTTON_COLOR ST77XX_WHITE
#define DISPLAY_MAIN_SCREEN_COLOR ST77XX_BLACK
#define DISPLAY_MAIN_SCREEN_OUTLINE_COLOR ST77XX_WHITE
#define DISPLAY_SONAR_DATA_COLOR ST77XX_BLUE

int connectionCounter = 0;

/// Battery icon colors
#define DISPLAY_BATTERY_BODY_COLOR ST77XX_WHITE
#define DISPLAY_BATTERY_BORDER_COLOR ST77XX_BLACK

#define DISPLAY_MAIN_SCREEN_WIDTH_MARGIN 80

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

enum DisplayScreens {
  NONE = 0,
  ERROR_NO_RADIO_CONNECTION = -1,
  ERROR_NOT_ENOUGH_SATELLITES = -2,
  GPS_SCREEN = 1,
  SONAR_SCREEN = 2
};

struct DisplayGpsSelectionData {
  uint8_t position;
  uint8_t nextPos;
  uint16_t rectX;
  uint16_t rectY;
};

struct DisplayState {
  DisplayScreens currentScreen;
  bool menuButtonPressed;
  bool selectButtonPressed;
  bool homeButtonPressed;
  bool holdingConfirmButton;
};

struct DisplayGpsPositionData {
  uint32_t longitudes[4];
  uint32_t latitudes[4];
  bool empty[4];
};

struct DisplayGpsSelectionData *currentGpsSelection;
struct DisplayState *displayState;
struct DisplayGpsPositionData gpsPositionData;

// Circular buffer to hold lake floor data
uint16_t displayLakeFloorBuffer[DISPLAY_WIDTH - DISPLAY_MAIN_SCREEN_WIDTH_MARGIN];
uint16_t displaySonarDrawIndex;

const uint16_t DISPLAY_UPDATE_FREQUENCY = 1 * 1000;
const uint16_t DISPLAY_SEND_TIME = 1 * 1000;
const uint16_t DISPLAY_SAVE_TIME = 2 * 1000;
const uint16_t DISPLAY_DELETE_TIME = 3 * 1000;
const uint16_t DISPLAY_CYCLE_ACTIONS_TIME = 4 * 1000;

unsigned long display_previousMillis = 0;
unsigned long display_sonarPreviousMillis = 0;
unsigned long display_topBarPreviousMillis = 0;
bool displayUpdateSonar = false;
bool displayUpdateTopBar = false;

double previousMilisReturnHome = 0;

/// Nfrc and i/o data
#pragma pack(push, 1)
struct commandData {
  int16_t xAxis;
  int16_t yAxis;
  int16_t leftDump;
  int16_t rightDump;
  uint16_t gpsGoToPosLat1;
  uint16_t gpsGoToPosLat2;
  uint16_t gpsGoToPosLon1;  //Ak su 0 tak nechod ak sa zmenia tak chod
  uint16_t gpsGoToPosLon2;
  bool returnHome;  //return Home if needed
};

#pragma pack(pop)

#pragma pack(push, 1)
struct SensorData {
  uint16_t sonarDistance;
  uint8_t sonarFIshFoundNum;
  uint16_t actualGpsPositionLon1;
  uint16_t actualGpsPositionLon2;
  uint16_t actualGpsPositionLat1;
  uint16_t actualGpsPositionLat2;
  uint8_t NumOfSats;
  bool coldStart;
  uint8_t batteryTeensy;
};
#pragma pack(pop)

SensorData dataReceived;
commandData dataToSend;

const int pinDriveYAxis = A0;
const int pinDriveXAxis = A1;
const int pinDumpLeft = A2;   //normally 7
const int pinDumpRight = A3;  //normally 8
//const int pinBattery = A17;

uint32_t rLat = 0;
uint32_t rLon = 0;

bool firstInitTime = false;

bool isSending = true;                       // Start in sending mode
unsigned long lastSwitchTime = 0;            // Last time we switched mode
const unsigned long sendingDuration = 1000;  // Send data for 1 second
bool awaitingData = false;                   // Flag to indicate awaiting reception of one data packet
bool radioAvailable = false;

//pins for NRF24L01
const int pinCE = 6;   // 9 normally
const int pinCSN = 7;  //10 normally

RF24 radio(pinCE, pinCSN);  // CE, CSN pins configuration
const byte addresses[][6] = { "1Node", "2Node" };

SPISettings settingsDevice1(4000000, MSBFIRST, SPI_MODE0);  // Example settings for device 1
SPISettings settingsDevice2(2000000, MSBFIRST, SPI_MODE1);  // Example settings for device 2

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MIN);
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]);
  radio.startListening();  // Start in listening mode to be ready to switch to sending mode
  Serial.println("Arduino is ready");

  pinMode(pinDumpLeft, INPUT_PULLUP);
  pinMode(pinDumpRight, INPUT_PULLUP);
  pinMode(pinCSN, OUTPUT);
  pinMode(TFT_CS, OUTPUT);

  retrieveDataFromEEPROM(gpsPositionData);
  initDisplay(GPS_SCREEN);
}

void loop() {

  if (!firstInitTime) {
    lastSwitchTime = millis();
    firstInitTime = true;
  }

  digitalWrite(pinCSN, LOW);
  SPI.endTransaction();
  //digitalWrite(TFT_CS, LOW);
  //SPI.beginTransaction(settingsDevice1);

  unsigned long currentTime = millis();

  if (isSending && currentTime - lastSwitchTime > sendingDuration) {
    // After sending for 1 second, switch to listening mode
    isSending = false;
    awaitingData = true;
    radio.startListening();
    //digitalWrite(pinCSN, LOW);
    lastSwitchTime = currentTime;  // Update the switch time
    Serial.println("Switching to listening mode...");
  }

  if (isSending) {
    // Create some data to send
    dataToSend.yAxis = analogRead(pinDriveYAxis);
    dataToSend.xAxis = analogRead(pinDriveXAxis);
    dataToSend.leftDump = digitalRead(pinDumpLeft);
    dataToSend.rightDump = digitalRead(pinDumpRight);

    if(dataToSend.returnHome){
      if(millis() - previousMilisReturnHome > 3000){
        dataToSend.returnHome = false;
      }
      
    }
    //Serial.println(dataToSend.rightDump);

    radio.stopListening();                         // Ensure we're not in listening mode
    radio.write(&dataToSend, sizeof(dataToSend));  // Send data

    digitalWrite(pinCSN, HIGH);
    SPI.endTransaction();

    //Serial.println("Sending data...");
    // delay(10); // Small delay to avoid spamming too fast
  }

  // Listen for one piece of data and switch back to sending
  if (!isSending && awaitingData) {

    if (radio.available()) {
      radio.read(&dataReceived, sizeof(dataReceived));
      radioAvailable = true;
      connectionCounter = 0;
      awaitingData = false;       // Reset flag after receiving data
      isSending = true;           // Switch back to sending mode
      lastSwitchTime = millis();  // Update the switch time
      Serial.print(dataReceived.sonarDistance);
      Serial.print("  ");
      Serial.print(dataReceived.sonarFIshFoundNum);
      Serial.print("  ");
      Serial.print(dataReceived.actualGpsPositionLat1);
      Serial.print("  ");
      Serial.print(dataReceived.actualGpsPositionLat2);
      Serial.print("  ");
      Serial.print(dataReceived.actualGpsPositionLon1);
      Serial.print("  ");
      Serial.print(dataReceived.actualGpsPositionLon2);
      Serial.print("  ");
      Serial.print(dataReceived.batteryTeensy);
      Serial.print("  ");
      Serial.println(dataReceived.NumOfSats);

      uint32_t reconstructedLattitude1 = 0;
      uint32_t reconstructedLattitude2 = 0;
      uint32_t reconstructedLongtitude1 = 0;
      uint32_t reconstructedLongtitude2 = 0;

      for (int l = 0; l < 4; l++) {
        //Serial.println(size(dataReceived.actualGpsPositionLat1));
        String str = String(dataReceived.actualGpsPositionLat1);
        char c = str[l];
        int tempC = c - '0';
        if (l > 0)
          reconstructedLattitude1 = concatenateDigitsString(reconstructedLattitude1, tempC);
        else
          reconstructedLattitude1 = tempC;
      }

      for (int l = 0; l < 5; l++) {
        String str = String(dataReceived.actualGpsPositionLat2);
        char c = str[l];
        int tempC = c - '0';
        if (l > 0)
          reconstructedLattitude2 = concatenateDigitsString(reconstructedLattitude2, tempC);
        else
          reconstructedLattitude2 = concatenateDigitsString(reconstructedLattitude2, tempC);
      }

      for (int l = 0; l < 4; l++) {
        //Serial.println(size(dataReceived.actualGpsPositionLat1));
        String str = String(dataReceived.actualGpsPositionLon1);
        char c = str[l];
        int tempC = c - '0';
        if (l > 0)
          reconstructedLongtitude1 = concatenateDigitsString(reconstructedLongtitude1, tempC);
        else
          reconstructedLongtitude1 = tempC;
      }

      for (int l = 0; l < 5; l++) {
        String str = String(dataReceived.actualGpsPositionLon2);
        char c = str[l];
        int tempC = c - '0';
        if (l > 0)
          reconstructedLongtitude2 = concatenateDigitsString(reconstructedLongtitude2, tempC);
        else
          reconstructedLongtitude2 = concatenateDigitsString(reconstructedLongtitude2, tempC);
      }

      rLat = concatenateDigitsString(reconstructedLattitude1, reconstructedLattitude2);
      rLon = concatenateDigitsString(reconstructedLongtitude1, reconstructedLongtitude2);

    } else {

      //TODO: Not connected to the boat
      if (!isSending) {
        if(connectionCounter > 100){
          radioAvailable = false;
        
        } else {
          connectionCounter++;
        }
      }
    }
  }

  digitalWrite(TFT_CS, LOW);
  SPI.beginTransaction(settingsDevice2);
  delay(10);
 // Serial.println(radioAvailable);
  updateDisplay(radioAvailable, displayState);
  digitalWrite(TFT_CS, HIGH);
  SPI.endTransaction();
  //delay(100);
}

void drawFishFinder(float sonarData, int lineThickness) {
  const uint8_t LAKE_FLOOR_HEIGHT = 20;
  const uint8_t TOP_BAR_MARGIN = 40;

  // Map sonar data to lake floor Y coordinate within the drawing area
  uint16_t lakeFloorY = map(sonarData, 0, SONAR_MAX_DISTANCE, tft.height() - 2, TOP_BAR_MARGIN);

  // Update the lake floor buffer at the current draw index
  displayLakeFloorBuffer[displaySonarDrawIndex] = lakeFloorY;

  // Calculate the height of the line from SCREEN_HEIGHT to lakeFloorY
  int lineHeight = tft.height() - 1 - lakeFloorY;

  // Draw vertical line with specified line thickness from SCREEN_HEIGHT to lakeFloorY at the current draw index
  for (int i = 0; i < lineThickness; i++) {
    tft.drawFastVLine(displaySonarDrawIndex + i + DISPLAY_MAIN_SCREEN_WIDTH_MARGIN, lakeFloorY, lineHeight, DISPLAY_SONAR_DATA_COLOR);
  }

  // Increment draw index
  displaySonarDrawIndex += lineThickness;

  // Check if draw index has reached or exceeded the screen width
  if (displaySonarDrawIndex >= tft.width() - DISPLAY_MAIN_SCREEN_WIDTH_MARGIN) {
    // Reset draw index to start drawing from the beginning of the screen
    displaySonarDrawIndex = 0;

    uint16_t rectX = DISPLAY_MAIN_SCREEN_WIDTH_MARGIN;
    uint16_t rectY = 40;
    uint16_t rectWidth = tft.width() - DISPLAY_MAIN_SCREEN_WIDTH_MARGIN;
    uint16_t rectHeight = tft.height() - 40;
    tft.fillRect(rectX, rectY, rectWidth, rectHeight, ST77XX_BLACK);
  }
}

// Helper function to determine battery fill color based on charge level
uint16_t getBatteryFillColor(uint8_t batteryCharge) {
  if (batteryCharge >= 50) {
    return ST77XX_GREEN;
  } else if (batteryCharge >= 25) {
    return ST77XX_ORANGE;
  } else if (batteryCharge >= 10) {
    return ST77XX_RED;
  } else {
    return ST77XX_RED;
  }
}

void drawBattery(uint16_t batteryWidth, uint16_t batteryHeight, uint16_t batteryMargin, uint16_t batteryX, uint16_t batteryY, uint8_t batteryCharge) {
  const uint16_t fillColor = getBatteryFillColor(batteryCharge);

  /// Draw battery body
  tft.fillRect(batteryX, batteryY, batteryWidth, batteryHeight, DISPLAY_BATTERY_BODY_COLOR);

  /// Calculate fill width based on battery charge
  uint16_t fillWidth = map(batteryCharge, 0, 100, 0, batteryWidth - 2 * batteryMargin);

  /// Draw battery fill level with determined color
  tft.fillRect(batteryX + batteryMargin, batteryY + batteryMargin, fillWidth, batteryHeight - 2 * batteryMargin, fillColor);

  /// Draw battery border
  tft.drawRect(batteryX, batteryY, batteryWidth, batteryHeight, DISPLAY_BATTERY_BORDER_COLOR);
}

void updateTopBarBackground() {
  uint8_t rectX = 90;
  uint8_t rectY = 0;
  uint8_t rectWidth = 100;
  uint8_t rectHeight = 15;
  tft.fillRect(rectX, rectY, rectWidth, rectHeight, DISPLAY_TOP_BAR_COLOR);
}

void updateTopBar(bool connected, uint8_t numberOfSatellites, DisplayScreens currentScreen) {
  if (!displayUpdateTopBar) {
    display_topBarPreviousMillis = millis();
    displayUpdateTopBar = true;
  }

  unsigned long topBarInterval = millis() - display_topBarPreviousMillis;

  if (topBarInterval >= DISPLAY_UPDATE_FREQUENCY) {
    displayUpdateTopBar = false;

    /// Satellites
    tft.setCursor(0, 2);
    tft.setTextSize(1);

    const uint8_t idealNumOfSatellites = 7;
    const uint8_t usableNumOfSatellites = 5;

    if (numberOfSatellites < usableNumOfSatellites) {
      tft.setTextColor(ST77XX_MAGENTA);  // bieda
    } else if (numberOfSatellites < idealNumOfSatellites && numberOfSatellites >= usableNumOfSatellites) {
      tft.setTextColor(ST77XX_ORANGE);  // take da sa
    } else {
      tft.setTextColor(ST77XX_GREEN);  // dobre
    }

    tft.fillRect(0, 0, 84, 10, ST77XX_BLACK);
    tft.print("Satellites: ");
    tft.print(numberOfSatellites);

    /// Connection Status
    tft.setCursor(72, 10);
    if (connected) {
      tft.setTextColor(ST77XX_GREEN);
      tft.print("OK");
    } else {
      tft.setTextColor(ST77XX_MAGENTA);
      tft.print("NO");
    }

    /// Draw battery icons
    drawBattery(20, 8, 2, tft.width() - 24, 2, dataReceived.batteryTeensy);  // Boat Battery
    drawBattery(20, 8, 2, tft.width() - 24, 10, 70);                         // Controller Battery

    /// Current Screen
    switch (currentScreen) {
      case GPS_SCREEN:
        tft.setCursor(100, 2);
        tft.setTextSize(1);
        tft.setTextColor(ST77XX_WHITE);
        tft.print("GPS Menu");
        break;

      case SONAR_SCREEN:
        tft.setCursor(100, 2);
        tft.setTextSize(1);
        tft.setTextColor(ST77XX_WHITE);
        tft.print("SONAR Menu");
        break;

      default:
        tft.setCursor(100, 2);
        tft.setTextSize(1);
        tft.setTextColor(ST77XX_WHITE);
        tft.print("Error");
        break;
    }
  }
}

void drawTopBar() {
  int rectX = 0;
  int rectY = 0;
  int rectWidth = tft.width();
  int rectHeight = 20;
  tft.fillRect(rectX, rectY, rectWidth, rectHeight, DISPLAY_TOP_BAR_COLOR);

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);

  /// Boat Battery
  tft.setCursor(tft.width() - 56, 2);
  tft.print("Boat: ");
  tft.println();

  /// Connection status
  tft.print("Connection: ");

  /// Joystick battery
  tft.setCursor(tft.width() - DISPLAY_MAIN_SCREEN_WIDTH_MARGIN, 10);
  tft.print("Controller: ");
}

void drawMenuBarWithButtons() {
  /// Menu bar
  uint16_t rectX = 0;
  uint16_t rectY = 20;
  uint16_t rectWidth = DISPLAY_MAIN_SCREEN_WIDTH_MARGIN;
  uint16_t rectHeight = tft.height();
  tft.fillRect(rectX, rectY, rectWidth, rectHeight, DISPLAY_MENU_BAR_COLOR);
  tft.drawRect(rectX, rectY, rectWidth, rectHeight, DISPLAY_MAIN_SCREEN_OUTLINE_COLOR);

  /// Buttons with text
  const uint8_t BUTTON_WIDTH = 60;
  const uint8_t BUTTON_HEIGHT = 30;

  /// MENU Button
  tft.drawRect(rectX + 10, rectY + 20, BUTTON_WIDTH, BUTTON_HEIGHT, DISPLAY_MENU_BAR_BUTTON_COLOR);

  tft.setCursor(rectX + 14, rectY + 24);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("MENU");
  rectY = rectY + 20 + BUTTON_HEIGHT;

  /// SELECT Button
  tft.drawRect(rectX + 10, rectY + 20, BUTTON_WIDTH, BUTTON_HEIGHT, DISPLAY_MENU_BAR_BUTTON_COLOR);

  tft.setCursor(rectX + 14, rectY + 24);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("SELECT");
  rectY = rectY + 20 + BUTTON_HEIGHT;

  /// CONFIRM Button
  tft.drawRect(rectX + 10, rectY + 20, BUTTON_WIDTH, BUTTON_HEIGHT, DISPLAY_MENU_BAR_BUTTON_COLOR);

  tft.setCursor(rectX + 14, rectY + 24);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("CONFIRM");
  rectY = rectY + 20 + BUTTON_HEIGHT;

  /// HOME Button
  tft.drawRect(rectX + 10, rectY + 20, BUTTON_WIDTH, BUTTON_HEIGHT, DISPLAY_MENU_BAR_BUTTON_COLOR);

  tft.setCursor(rectX + 14, rectY + 24);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("HOME");
  tft.setCursor(rectX + 14, rectY + 24 + 8);
  rectY = rectY + 20 + BUTTON_HEIGHT;
}

void drawMainScreenBackground(uint16_t backgroundColor, DisplayScreens screen) {
  uint16_t rectX = DISPLAY_MAIN_SCREEN_WIDTH_MARGIN;
  uint16_t rectY = 20;
  uint16_t rectWidth = tft.width();
  uint16_t rectHeight = tft.height() - 20;

  switch (screen) {
    case SONAR_SCREEN:
      tft.fillRect(rectX, rectY, rectWidth, rectHeight, backgroundColor);
      tft.drawLine(rectX, rectY, rectWidth, rectY, DISPLAY_MAIN_SCREEN_OUTLINE_COLOR);
      break;
    case GPS_SCREEN:
      tft.fillRect(rectX, rectY, rectWidth, rectHeight, backgroundColor);
      tft.drawRect(rectX, rectY, rectWidth, rectHeight, DISPLAY_MAIN_SCREEN_OUTLINE_COLOR);
      break;
  }
}

void drawErrorScreen(DisplayScreens screen) {
  uint16_t rectX = 0;
  uint16_t rectY = 0;
  uint16_t rectWidth = tft.width();
  uint16_t rectHeight = tft.height();

  switch (screen) {
    case ERROR_NO_RADIO_CONNECTION:
      /// Fill background
      tft.fillRect(rectX, rectY, rectWidth, rectHeight, ST77XX_BLACK);
      tft.drawRect(rectX, rectY, rectWidth, rectHeight, ST77XX_RED);

      /// Draw inner rectangle
      tft.fillRect(rectX + 20, rectY + 20, rectWidth - 40, rectHeight - 40, ST77XX_BLACK);
      tft.drawRect(rectX + 20, rectY + 20, rectWidth - 40, rectHeight - 40, ST77XX_RED);

      rectX = rectX + 20;
      rectY = rectY + 20;

      /// Display error message
      tft.setTextColor(ST77XX_RED);
      tft.setTextSize(3);

      tft.setCursor(rectX + 90, rectY + 60);
      tft.print("ERROR!");
      tft.setCursor(rectX + 16, rectY + 20 + 80);
      tft.print("NO CONNECTION!");
      break;
    case ERROR_NOT_ENOUGH_SATELLITES:
      /// Fill background
      tft.fillRect(rectX, 20, rectWidth, rectHeight, ST77XX_BLACK);

      rectX = rectX + 20;
      rectY = rectY + 20;

      /// Display error message
      tft.setTextColor(ST77XX_RED);
      tft.setTextSize(2);

      tft.setCursor(rectX + 10, rectY + 60);
      tft.print("NOT ENOUGH SATELLITES!");
      tft.setCursor(rectX + 24, rectY + 20 + 80);
      tft.print("USE MANUAL CONTROL!");
      break;
  }
}

void selectMainScreenGps(struct DisplayGpsSelectionData *selectionState) {
  uint8_t rectX = selectionState->rectX;
  uint8_t rectY = selectionState->rectY;

  if (selectionState->position == 1)
    tft.drawRect(rectX, rectY + 3 * 20 + 3 * DISPLAY_BUTTON_HEIGHT, DISPLAY_BUTTON_WIDTH, DISPLAY_BUTTON_HEIGHT, DISPLAY_MAIN_SCREEN_OUTLINE_COLOR);
  else
    tft.drawRect(rectX, rectY - 20 - DISPLAY_BUTTON_HEIGHT, DISPLAY_BUTTON_WIDTH, DISPLAY_BUTTON_HEIGHT, DISPLAY_MAIN_SCREEN_OUTLINE_COLOR);

  /// Highlight
  tft.drawRect(rectX, rectY, DISPLAY_BUTTON_WIDTH, DISPLAY_BUTTON_HEIGHT, ST77XX_YELLOW);

  selectionState->rectX = rectX;
  selectionState->rectY = rectY;
}

void updateMainScreenGpsValues() {
  const uint8_t CURSOR_NEW_LINE = 10;
  const uint8_t NUM_POSITIONS = 4;

  uint16_t rectX = 20 + DISPLAY_MAIN_SCREEN_WIDTH_MARGIN;
  uint16_t rectY = 20;
  uint16_t rectWidth = tft.width();
  uint16_t rectHeight = tft.height();

  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);

  for (uint8_t i = 0; i < NUM_POSITIONS; i++) {
    tft.setCursor(rectX + 14, rectY + 24 + CURSOR_NEW_LINE);

    if (gpsPositionData.empty[i]) {
      tft.print("EMPTY");
    } else {
      tft.print("SAVED");
    }

    rectY += 20 + DISPLAY_BUTTON_HEIGHT;
  }
}

void updateMainScreenSonarValues() {
  if (!displayUpdateSonar) {
    display_sonarPreviousMillis = millis();
    displayUpdateSonar = true;
  }

  unsigned long sonarInterval = millis() - display_sonarPreviousMillis;

  if (sonarInterval >= DISPLAY_UPDATE_FREQUENCY) {
    //float sonarData = random(0, 600); // Random value between 0 and 600 (replace with real data)
    float sonarData = 610 - dataReceived.sonarDistance;

    // Draw fish finder point based on sonar data
    drawFishFinder(sonarData, 2);
    displayUpdateSonar = false;

    uint16_t rectX = 20 + DISPLAY_MAIN_SCREEN_WIDTH_MARGIN;
    uint16_t rectY = 40;
    uint16_t rectWidth = tft.width();
    uint16_t rectHeight = tft.height() - 20;

    const uint8_t NUM_POINTS = 5;
    const uint8_t POINT_VALUES[NUM_POINTS] = { 25, 50, 75, 100, 0 };  // Values to map
    uint16_t lakeFloorY[NUM_POINTS];                                  // Array to store mapped Y coordinates

    // Calculate mapped Y coordinates for each point
    for (int i = 0; i < NUM_POINTS; i++) {
      lakeFloorY[i] = map(POINT_VALUES[i], 0, 100, rectHeight, rectY);
    }

    // Draw lines for each mapped Y coordinate
    for (int i = 0; i < NUM_POINTS; i++) {
      tft.drawLine(rectX - 5, lakeFloorY[i], rectX + 5, lakeFloorY[i], ST77XX_WHITE);
    }

    // Draw a vertical line on the left side of the display
    tft.drawLine(rectX, rectY, rectX, rectHeight, ST77XX_WHITE);

    tft.setCursor(4 + DISPLAY_MAIN_SCREEN_WIDTH_MARGIN, tft.height() - 10);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(1);

    tft.print(SONAR_MAX_DISTANCE);
    tft.print(" cm");

    /// Fish number
    tft.fillRect(tft.width() - 45, 32, 20, 10, ST77XX_BLACK);  // Clear number
    uint8_t numberOfFish = dataReceived.sonarFIshFoundNum;
    tft.setCursor(tft.width() - 40, 24);
    tft.print("Fish: ");
    tft.setCursor(tft.width() - 40, 32);
    tft.print(numberOfFish);
  }
}

void drawMainScreenSonar() {
  drawMainScreenBackground(ST77XX_BLACK, SONAR_SCREEN);
}

void drawMainScreenGps() {
  drawMainScreenBackground(ST77XX_BLACK, GPS_SCREEN);
  uint16_t rectX = 20 + DISPLAY_MAIN_SCREEN_WIDTH_MARGIN;
  uint16_t rectY = 20;
  uint16_t rectWidth = tft.width();
  uint16_t rectHeight = tft.height();

  /// GPS Position 1
  tft.drawRect(rectX + 10, rectY + 20, DISPLAY_BUTTON_WIDTH, DISPLAY_BUTTON_HEIGHT, DISPLAY_MAIN_SCREEN_OUTLINE_COLOR);
  tft.setCursor(rectX + 14, rectY + 24);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("GPS POSITION 1");
  rectY = rectY + 20 + DISPLAY_BUTTON_HEIGHT;

  /// GPS Position 2
  tft.drawRect(rectX + 10, rectY + 20, DISPLAY_BUTTON_WIDTH, DISPLAY_BUTTON_HEIGHT, DISPLAY_MAIN_SCREEN_OUTLINE_COLOR);
  tft.setCursor(rectX + 14, rectY + 24);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("GPS POSITION 2");
  rectY = rectY + 20 + DISPLAY_BUTTON_HEIGHT;

  /// GPS Position 3
  tft.drawRect(rectX + 10, rectY + 20, DISPLAY_BUTTON_WIDTH, DISPLAY_BUTTON_HEIGHT, DISPLAY_MAIN_SCREEN_OUTLINE_COLOR);
  tft.setCursor(rectX + 14, rectY + 24);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("GPS POSITION 3");
  rectY = rectY + 20 + DISPLAY_BUTTON_HEIGHT;

  /// GPS Position 4
  tft.drawRect(rectX + 10, rectY + 20, DISPLAY_BUTTON_WIDTH, DISPLAY_BUTTON_HEIGHT, DISPLAY_MAIN_SCREEN_OUTLINE_COLOR);
  tft.setCursor(rectX + 14, rectY + 24);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("GPS POSITION 4");
  rectY = rectY + 20 + DISPLAY_BUTTON_HEIGHT;
}

void setGpsScreenAsCurrentScreen(struct DisplayState *state) {
  state->currentScreen = GPS_SCREEN;
  drawMainScreenGps();

  currentGpsSelection->position = 1;
  currentGpsSelection->rectX = 30 + DISPLAY_MAIN_SCREEN_WIDTH_MARGIN;
  currentGpsSelection->rectY = 40;
  selectMainScreenGps(currentGpsSelection);
  currentGpsSelection->nextPos = 2;

  displaySonarDrawIndex = 0;
}

void handleMenuButton(struct DisplayState *state) {
  uint8_t menuButtonState = digitalRead(DISPLAY_BUTTON_MENU);

  if (menuButtonState == LOW && !state->menuButtonPressed) {

    if (state->currentScreen == GPS_SCREEN) {
      state->currentScreen = SONAR_SCREEN;
      drawMainScreenSonar();

    } else if (state->currentScreen == SONAR_SCREEN) {
      setGpsScreenAsCurrentScreen(state);
    }
    state->menuButtonPressed = true;
  } else if (menuButtonState == HIGH) {
    state->menuButtonPressed = false;
  }
}

void handleSelectionButton(struct DisplayState *state, struct DisplayGpsSelectionData *currentGpsSelection) {
  uint8_t selectionButtonState = digitalRead(DISPLAY_BUTTON_SELECT);

  if (selectionButtonState == LOW && state->currentScreen == GPS_SCREEN && !state->selectButtonPressed) {
    switch (currentGpsSelection->nextPos) {
      case 1:
        currentGpsSelection->position = 1;
        currentGpsSelection->rectX = 30 + DISPLAY_MAIN_SCREEN_WIDTH_MARGIN;
        currentGpsSelection->rectY = 40;
        selectMainScreenGps(currentGpsSelection);
        currentGpsSelection->nextPos += 1;
        break;
      case 2:
      case 3:
      case 4:
        currentGpsSelection->position += 1;
        currentGpsSelection->rectY += 20 + DISPLAY_BUTTON_HEIGHT;
        selectMainScreenGps(currentGpsSelection);
        currentGpsSelection->nextPos = (currentGpsSelection->nextPos % 4) + 1;
        break;
    }
    state->selectButtonPressed = true;
  } else if (selectionButtonState == HIGH) {
    state->selectButtonPressed = false;
  }
}

void handleConfirmButton(struct DisplayState *state, struct DisplayGpsSelectionData *currentGpsSelection) {
  if (!state->currentScreen == GPS_SCREEN) {
    return;
  }

  uint8_t confirmButtonState = digitalRead(DISPLAY_BUTTON_CONFIRM);
  unsigned long display_interval = 100 * 1000;

  /// Visualization ----------------
  if (state->holdingConfirmButton == true && confirmButtonState == LOW) {  // Ak sa drzi tlacidlo
    display_interval = millis() - display_previousMillis;
    if (display_interval > DISPLAY_CYCLE_ACTIONS_TIME) {
      display_previousMillis = millis();
    }

    /// SEND TO POS action
    if (display_interval <= DISPLAY_SEND_TIME) {
      tft.drawRect(currentGpsSelection->rectX, currentGpsSelection->rectY, DISPLAY_BUTTON_WIDTH, DISPLAY_BUTTON_HEIGHT, ST77XX_GREEN);
    }
    /// SAVING Action
    if (display_interval > DISPLAY_SEND_TIME && display_interval <= DISPLAY_SAVE_TIME) {
      tft.drawRect(currentGpsSelection->rectX, currentGpsSelection->rectY, DISPLAY_BUTTON_WIDTH, DISPLAY_BUTTON_HEIGHT, ST77XX_BLUE);
    }
    /// DELETE Action
    if (display_interval > DISPLAY_SAVE_TIME && display_interval <= DISPLAY_DELETE_TIME) {
      tft.drawRect(currentGpsSelection->rectX, currentGpsSelection->rectY, DISPLAY_BUTTON_WIDTH, DISPLAY_BUTTON_HEIGHT, ST77XX_RED);
    }
  }

  if (confirmButtonState == LOW && state->holdingConfirmButton == false) {
    state->holdingConfirmButton = true;
    display_previousMillis = millis();
  }

  if (state->holdingConfirmButton == true && confirmButtonState == HIGH) {
    state->holdingConfirmButton = false;

    display_interval = millis() - display_previousMillis;
    drawMainScreenGps();

    if (display_interval <= DISPLAY_SEND_TIME) {  /// SEND TO POS Action
      uint32_t temp2 = gpsPositionData.longitudes[currentGpsSelection->position - 1];
      uint32_t temp1 = gpsPositionData.latitudes[currentGpsSelection->position - 1];
      dataToSend.gpsGoToPosLat1 = round(temp1 / 100000);
      dataToSend.gpsGoToPosLat2 = round(temp1 % 100000);
      dataToSend.gpsGoToPosLon1 = round(temp2 / 100000);
      dataToSend.gpsGoToPosLon2 = round(temp2 % 100000);            

    } else if (display_interval > DISPLAY_SEND_TIME && display_interval <= DISPLAY_SAVE_TIME) {  /// SAVE Action
      gpsPositionData.longitudes[currentGpsSelection->position - 1] = rLon;
      gpsPositionData.latitudes[currentGpsSelection->position - 1] = rLat;
      gpsPositionData.empty[currentGpsSelection->position - 1] = false;      
      // Store the data in EEPROM
      storeDataInEEPROM(gpsPositionData);

    } else if (display_interval > DISPLAY_SAVE_TIME && display_interval <= DISPLAY_DELETE_TIME) {  /// DELETE Action
      gpsPositionData.longitudes[currentGpsSelection->position - 1] = 0;
      gpsPositionData.latitudes[currentGpsSelection->position - 1] = 0;
      gpsPositionData.empty[currentGpsSelection->position - 1] = true;
    }

    /// Reselect ------------
    selectMainScreenGps(currentGpsSelection);
  }
}

void handleHomeButton(struct DisplayState *state) {
  uint8_t homeButtonState = digitalRead(DISPLAY_BUTTON_HOME);

  if (homeButtonState == LOW && !state->homeButtonPressed) {
    Serial.println("Going home!");
    dataToSend.returnHome = true;
    previousMilisReturnHome = millis();
    state->homeButtonPressed = true;
  } else if (homeButtonState == HIGH) {
    state->homeButtonPressed = false;
  }
}

void updateDisplay(bool connected, struct DisplayState *state) {
  DisplayScreens screen = state->currentScreen;

  if (screen == GPS_SCREEN || screen == SONAR_SCREEN) {
    handleMenuButton(state);
    handleSelectionButton(state, currentGpsSelection);
    handleConfirmButton(state, currentGpsSelection);
    handleHomeButton(state);
  }

  uint8_t numberOfSatellites = dataReceived.NumOfSats;

  /// Update values ================
  if (connected) {

    if (state->currentScreen == ERROR_NO_RADIO_CONNECTION) {
      tft.fillScreen(ST77XX_BLACK);
      drawTopBar();
      drawMenuBarWithButtons();
      setGpsScreenAsCurrentScreen(state);
    } 

    if (screen != state->currentScreen)
      updateTopBarBackground();

    updateTopBar(connected, numberOfSatellites, state->currentScreen);

    if (numberOfSatellites > 3) {

      if (state->currentScreen == ERROR_NOT_ENOUGH_SATELLITES) {
        tft.fillScreen(ST77XX_BLACK);
        drawTopBar();
        drawMenuBarWithButtons();
        setGpsScreenAsCurrentScreen(state);
      }

      if (state->currentScreen == GPS_SCREEN) {
        updateMainScreenGpsValues();
      } else if (state->currentScreen == SONAR_SCREEN) {
        updateMainScreenSonarValues();
      }

     } else if (numberOfSatellites < 4 && state->currentScreen != ERROR_NOT_ENOUGH_SATELLITES) {
      state->currentScreen = ERROR_NOT_ENOUGH_SATELLITES;
      updateTopBarBackground();
      drawErrorScreen(ERROR_NOT_ENOUGH_SATELLITES);
    }

  } else if (!connected && state->currentScreen != ERROR_NO_RADIO_CONNECTION) {
    state->currentScreen = ERROR_NO_RADIO_CONNECTION;
    drawErrorScreen(ERROR_NO_RADIO_CONNECTION);
  }
}

void initDisplay(DisplayScreens defaultScreen) {
  /// Init Pins
  pinMode(DISPLAY_BUTTON_MENU, INPUT_PULLUP);
  pinMode(DISPLAY_BUTTON_SELECT, INPUT_PULLUP);
  pinMode(DISPLAY_BUTTON_CONFIRM, INPUT_PULLUP);
  pinMode(DISPLAY_BUTTON_HOME, INPUT_PULLUP);

  /// Init display
  tft.init(240, 320);
  tft.setRotation(1);        /// To ensure that 0, 0 is in the top left corner...
  tft.invertDisplay(false);  /// for some reason the default of the display is to be inverted...
  tft.fillScreen(ST77XX_BLACK);

  drawTopBar();
  drawMenuBarWithButtons();

  /// Init structs
  displayState = new DisplayState;
  if (displayState == nullptr) {
    // Memory allocation failed
  } else {
    // Memory allocation successful
    displayState->currentScreen = NONE;
    displayState->menuButtonPressed = false;
    displayState->selectButtonPressed = false;
    displayState->homeButtonPressed = false;
    displayState->holdingConfirmButton = false;
  }

  currentGpsSelection = new DisplayGpsSelectionData;
  if (currentGpsSelection == nullptr) {
    // Memory allocation failed
  } else {
    // Memory allocation successful
    currentGpsSelection->position = 1;
    currentGpsSelection->nextPos = 2;
    currentGpsSelection->rectX = 30 + DISPLAY_MAIN_SCREEN_WIDTH_MARGIN;
    currentGpsSelection->rectY = 40;
  }

  /// Initialize lake floor buffer with default values (bottom of the screen)
  for (uint16_t i = 0; i < tft.width() - DISPLAY_MAIN_SCREEN_WIDTH_MARGIN; i++) {
    displayLakeFloorBuffer[i] = tft.height();
  }

  for (uint8_t i = 0; i < sizeof(gpsPositionData.empty); i++) {
    gpsPositionData.empty[i] = true;
    gpsPositionData.latitudes[i] = 0;
    gpsPositionData.longitudes[i] = 0;
  }

  switch (defaultScreen) {
    case GPS_SCREEN:
      drawMainScreenGps();
      displayState->currentScreen = GPS_SCREEN;
      selectMainScreenGps(currentGpsSelection);
      currentGpsSelection->nextPos = 2;
      break;

    case SONAR_SCREEN:
      drawMainScreenSonar();
      displayState->currentScreen = SONAR_SCREEN;
      break;

    default:
      displayState->currentScreen = NONE;
      break;
  }
}

uint32_t concatenateDigitsString(uint32_t digit1, uint32_t digit2) {
  // Convert digits to strings
  String str1 = String(digit1);
  String str2 = String(digit2);

  // Serial.println(str1);
  // Serial.println(str2);

  // Concatenate the strings
  String concatenatedStr = str1 + str2;
  // Serial.println(concatenatedStr);

  // Convert the concatenated string back to an integer
  uint32_t concatenatedInt = concatenatedStr.toInt();
  return concatenatedInt;
}

void storeDataInEEPROM(const DisplayGpsPositionData &data) {
  int address = 0;
  
  // Store longitudes
  for (uint8_t i = 0; i < 4; i++) {
    EEPROM.put(address, data.longitudes[i]);
    address += sizeof(data.longitudes[i]);
  }

  // Store latitudes
  for (uint8_t i = 0; i < 4; i++) {
    EEPROM.put(address, data.latitudes[i]);
    address += sizeof(data.latitudes[i]);
  }

  // Store empty flags
  for (uint8_t i = 0; i < 4; i++) {
    EEPROM.put(address, data.empty[i]);
    address += sizeof(data.empty[i]);
  }
}

void retrieveDataFromEEPROM(DisplayGpsPositionData &data) {
  int address = 0;

  // Retrieve longitudes
  for (uint8_t i = 0; i < 4; i++) {
    EEPROM.get(address, data.longitudes[i]);
    address += sizeof(data.longitudes[i]);
  }

  // Retrieve latitudes
  for (uint8_t i = 0; i < 4; i++) {
    EEPROM.get(address, data.latitudes[i]);
    address += sizeof(data.latitudes[i]);
  }

  // Retrieve empty flags
  for (uint8_t i = 0; i < 4; i++) {
    EEPROM.get(address, data.empty[i]);
    address += sizeof(data.empty[i]);
  }
}

