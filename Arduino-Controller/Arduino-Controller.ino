#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>



struct commandData {
  int16_t xAxis;
  int16_t yAxis;
  int16_t leftDump;
  int16_t rightDump;
  double gpsGoToPosLat;
  double gpsGoToPosLon; //Ak su 0 tak nechod ak sa zmenia tak chod
  bool returnHome; //return Home if needed
};

struct SensorData{
  int16_t sonarDistance;
  int16_t sonarFIshFoundNum;
  double actualGpsPositionLon;
  double actualGpsPositionLat;
};

const int pinDriveYAxis = A0;
const int pinDriveXAxis = A1;
const int pinDumpLeft = 7;
const int pinDumpRight = 8;

bool isSending = true; // Start in sending mode
unsigned long lastSwitchTime = 0; // Last time we switched mode
const unsigned long sendingDuration = 1000; // Send data for 1 second
bool awaitingData = false; // Flag to indicate awaiting reception of one data packet

//pins for NRF24L01
const int pinCE = 9;
const int pinCSN = 10;

RF24 radio(pinCE, pinCSN); // CE, CSN pins configuration
const byte addresses[][6] = {"1Node", "2Node"};

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MIN);
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]);
  radio.startListening(); // Start in listening mode to be ready to switch to sending mode
  Serial.println("Arduino is ready");

  pinMode(pinDumpLeft, INPUT);
  pinMode(pinDumpRight, INPUT);
}

void loop() {
  unsigned long currentTime = millis();

  if (isSending && currentTime - lastSwitchTime > sendingDuration) {
    // After sending for 1 second, switch to listening mode
    isSending = false;
    awaitingData = true;
    radio.startListening();
    lastSwitchTime = currentTime; // Update the switch time
    Serial.println("Switching to listening mode...");
  }

  if (isSending) {
    // Create some data to send
    commandData dataToSend;
    dataToSend.yAxis = analogRead(pinDriveYAxis);
    dataToSend.xAxis = analogRead(pinDriveXAxis);
    dataToSend.leftDump = digitalRead(pinDumpLeft);
    dataToSend.rightDump = digitalRead(pinDumpRight);
    radio.stopListening(); // Ensure we're not in listening mode
    radio.write(&dataToSend, sizeof(dataToSend)); // Send data
    //Serial.println("Sending data...");
    delay(10); // Small delay to avoid spamming too fast
  }

  // Listen for one piece of data and switch back to sending
  if (!isSending && awaitingData) {
    if (radio.available()) {
      SensorData dataReceived;
      radio.read(&dataReceived, sizeof(dataReceived));
      awaitingData = false; // Reset flag after receiving data
      isSending = true; // Switch back to sending mode
      lastSwitchTime = millis(); // Update the switch time
      Serial.println(dataReceived.actualGpsPositionLat);
    }

    else{
      //TODO Not connected to the boat
    }
  }
}
