#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <TinyGPS++.h>
#include <math.h>

// 16/3/2024

#define deadZone 20
#define speedZone 20

#define slowSpeed 200  // PWM
#define fastSpeed 255

double desiredLat = 0;
double desiredLon = 0;

typedef struct {
  int x;
  int y;
} MiddlePoint;

typedef enum {
  N,
  NW,
  NE,
  E,
  W,
  SW,
  SE,
  S,
  None
} Direction;  //svetove strany


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
  bool returnHome;       //return Home if needed
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

SensorData dataToSend;
commandData dataToReceive;

// battery
const int batteryPin = A16;


//GPS
void getGPSPoint();
void gpsLoop();
double getNewestLat();
double getNewestLng();
void calculateGPSDistance();
double haversine(double lat1, double lon1, double lat2, double lon2);

//static const uint8_t GPS_RXpin = 4;
//static const uint8_t GPS_TXpin = 3;
static const long GPSbaud = 9600;

uint8_t gps_dataAvailable = 0;
uint8_t gps_hasCorrectData = 0;

double gps_point_lat_lon_array[4][2];
uint32_t gps_satelite_number = 0;
uint8_t gps_point_counter = 0; 
double gps_heading_degrees = 0.0;
double fullDistanceTraveled = 0;

TinyGPSPlus gps;
//SoftwareSerial Serial7(GPS_RXpin, GPS_TXpin);

unsigned long gps_previousMillis = 0UL;
unsigned long gps_interval = 1000UL;

unsigned long gps_previousMillis2 = 0UL;

//GPS END


////Senzor utrazvuk
#define ECHOPIN 17
#define TRIGPIN 18

unsigned long previousMillisSonar = 0;
const long intervalSonar = 100;
int previousDistance = -1;
unsigned long fishTimestamps[100];
int fishCount = 0;

unsigned long triggerTime = 0;
volatile unsigned long echoStartTime = 0;
volatile unsigned long echoEndTime = 0;
volatile bool echoReceived = false;





unsigned long previousMillisLeft = 0;   // Stores last time the functions were triggered
const long intervalLeft = 600;         // Interval at which to run the second function (milliseconds)
unsigned long previousMillisRight = 0;  // Stores last time the functions were triggered
const long intervalRight = 600;        // Interval at which to run the second function (milliseconds)
bool isTimerLeftDumpTriggered = false;
bool isTimerRightDumpTriggered = false;

// Pin setup for driving and dumping
const int pinDriveLeftDirection1 = 6;
const int pinDriveLeftDirection2 = 7;
const int pinDriveRightDirection1 = 8;
const int pinDriveRightDirection2 = 27;
const int pinDumpLeftDirection1 = 23;
const int pinDumpLeftDirection2 = 22;
const int pinDumpRightDirection1 = 21;
const int pinDumpRightDirection2 = 20;
const int pinPwmDriveLeft = 2;      // PWM control for left drive
const int pinPwmDriveRight = 3;     // PWM control for right drive
const int pinPwmDumpLeft = 4;       // PWM control for left dump
const int pinPwmDumpRight = 5;      // PWM control for right dump
const int pwmValueDumpLeft = 165;   // Set a PWM value between 0 (0% duty cycle) and 255 (100% duty cycle) //TODO zistit
const int pwmValueDumpRight = 165;  // Set a PWM value between 0 (0% duty cycle) and 255 (100% duty cycle)

//pins for NRF24L01
const int pinCE = 19;
const int pinCSN = 10;
RF24 radio(pinCE, pinCSN);  // Adjust pin numbers for CE, CSN to match your Teensy setup
const byte addresses[][6] = { "1Node", "2Node" };

int dumpLeftCounter = 0;
int dumpRightCounter = 0;

int previousDumpLeft = 1;
int previousDumoRight = 1;


void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1, addresses[0]);
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  radio.startListening();

  // Initialize motor and dump control pins
  pinMode(pinDriveLeftDirection1, OUTPUT);
  pinMode(pinDriveLeftDirection2, OUTPUT);
  pinMode(pinDriveRightDirection1, OUTPUT);
  pinMode(pinDriveRightDirection2, OUTPUT);
  pinMode(pinDumpLeftDirection1, OUTPUT);
  pinMode(pinDumpLeftDirection2, OUTPUT);
  pinMode(pinDumpRightDirection1, OUTPUT);
  pinMode(pinDumpRightDirection2, OUTPUT);
  pinMode(pinPwmDriveLeft, OUTPUT);
  pinMode(pinPwmDriveRight, OUTPUT);
  pinMode(pinPwmDumpLeft, OUTPUT);
  pinMode(pinPwmDumpRight, OUTPUT);

  //Battery
 // pinMode(A17, INPUT);

  //Ultrazvuk
  pinMode(ECHOPIN, INPUT_PULLUP);
  pinMode(TRIGPIN, OUTPUT);
  digitalWrite(ECHOPIN, HIGH);
  memset(fishTimestamps, 0, sizeof(fishTimestamps));

  //GPS
  Serial7.begin(9600);  // Initialize Serial6 at 9600 baud
}

void loop() {
  //sendDataToArduino();
  if (radio.available()) {
    radio.read(&dataToReceive, sizeof(commandData));
    Serial.print(dataToReceive.yAxis);
    Serial.print("  ");
    Serial.print(dataToReceive.xAxis);
    Serial.print("  ");
    Serial.print(isTimerLeftDumpTriggered);
    Serial.print("  ");
    Serial.println(isTimerRightDumpTriggered);

    // if(!dataToReceive.leftDump)
    // dumpLeftCounter++;
    // else
    // dumpLeftCounter = 0;

    // if(!dataToReceive.rightDump)
    // dumpRightCounter++;
    // else
    // dumpRightCounter = 0;

    if (!dataToReceive.leftDump || isTimerLeftDumpTriggered) {  // TODO If lost connection while dumping This makes Dump Motor Left to dump
      triggerFunctionsNonBlockingLeftDump();
    }

    if (!dataToReceive.rightDump || isTimerRightDumpTriggered) {  // TODO If lost connection while dumping This makes Dump Motor Right to dump
      triggerFunctionsNonBlockingRightDump();
    }

    desiredLat = concatenateDigitsString(dataToReceive.gpsGoToPosLat1, dataToReceive.gpsGoToPosLat2);
    desiredLon = concatenateDigitsString(dataToReceive.gpsGoToPosLon1, dataToReceive.gpsGoToPosLon2);
    desiredLat = desiredLat / 10000000;
    desiredLon = desiredLon / 10000000;
    drive(dataToReceive.xAxis, dataToReceive.yAxis);
  }

  else {
    //Serial.println("Radio Didnt Receive"); //TODO if radio didnt receive get time and if not for 15 sec then go to home.
  }
  
   
   //Sonar
   processSonar();
   // Sonar END

  //GPS
  processGps();
//Fake data
  // uint temp1 = round(48.5641198489998198156 *10000000);
  // uint temp2 = round(17.6519512951981595195 *10000000);

  //  dataToSend.actualGpsPositionLat1 = round(temp1 / 100000);
  // //  dataToSend.actualGpsPositionLat1 = 2;
  //  dataToSend.actualGpsPositionLat2 = round(temp1 % 100000);
  // //  dataToSend.actualGpsPositionLat2 = 41189;
  // dataToSend.actualGpsPositionLon1 = round(temp2 / 100000);
  // dataToSend.actualGpsPositionLon2 = round(temp2 % 100000);
  // Serial.print(dataToSend.actualGpsPositionLat1);
  //  Serial.print(" ");
  // Serial.print(dataToSend.actualGpsPositionLat2);
  // Serial.print(" ");
  //  Serial.print(dataToSend.actualGpsPositionLon1);
  //  Serial.print(" ");
  //  Serial.print(dataToSend.actualGpsPositionLon2);
  //  Serial.print(" ");
  //  Serial.println(dataToSend.NumOfSats);

  // uint32_t lattitude = 485641198;
  // double rLat = lattitude;
  // Serial.println(rLat);

  // Serial.println(dataToSend.actualGpsPositionLat);
  // Serial.println(dataToSend.actualGpsPositionLon);
  //GPS ENd

sendDataToArduino(); 

    delay(10);
}

void triggerFunctionsNonBlockingLeftDump() {
  unsigned long currentMillis = millis();

  if (!isTimerLeftDumpTriggered) {
    previousMillisLeft = currentMillis;  // Save the time of the trigger
    isTimerLeftDumpTriggered = true;
    analogWrite(pinPwmDumpLeft, pwmValueDumpLeft);
    digitalWrite(pinDumpLeftDirection1, 1);
    digitalWrite(pinDumpLeftDirection2, 0);
  }

  if (isTimerLeftDumpTriggered && currentMillis - previousMillisLeft >= intervalLeft) {
    isTimerLeftDumpTriggered = false;  // Reset the trigger flag
    analogWrite(pinPwmDumpLeft, 0);
    digitalWrite(pinDumpLeftDirection1, 0);
    digitalWrite(pinDumpLeftDirection2, 0);
  }
}

void triggerFunctionsNonBlockingRightDump() {
  unsigned long currentMillis = millis();

  if (!isTimerRightDumpTriggered) {
    previousMillisRight = currentMillis;  // Save the time of the trigger
    isTimerRightDumpTriggered = true;
    analogWrite(pinPwmDumpRight, pwmValueDumpRight);
    digitalWrite(pinDumpRightDirection1, 1);
    digitalWrite(pinDumpRightDirection2, 0);
  }

  if (isTimerRightDumpTriggered && currentMillis - previousMillisRight >= intervalRight) {
    isTimerRightDumpTriggered = false;  // Reset the trigger flag
    analogWrite(pinPwmDumpRight, 0);
    digitalWrite(pinDumpRightDirection1, 0);
    digitalWrite(pinDumpRightDirection2, 0);
  }
}
void sendDataToArduino() {

  radio.stopListening();
  dataToSend.sonarDistance = previousDistance;  // Prepare your data //TODO sonar, prejdena vzdialenost, gps data
  dataToSend.sonarFIshFoundNum = fishCount;
  dataToSend.batteryTeensy = map(((analogRead(batteryPin) * (3.3/1024)) * (1000+3000)/1000),11.1,12.6,0,100);
  // int sss = 2;
  // dataToSend.actualGpsPositionLat1 = previousDistance;
  // dataToSend.actualGpsPositionLat = 
  // dataToSend.actualGpsPositionLon = 
  //Serial.println("Sending");
  while (radio.write(&dataToSend, sizeof(SensorData))) {
    //Serial.println("Sending 1");
    //Keep sending until you acnowledge
    //TODO after 15 sec return home
  }

  radio.startListening();
}

///////////Drive Manual////////////////
void drive(int xData, int yData) {
  setMiddlePoint(500, 500);
  Direction direction = setMovement(xData, yData);
  //Serial.println(direction);
  int speed = setSpeed(xData);
  static int speedR;
  static int speedL;
  int motorPinAR = 1;
  int motorPinBR = 0;
  int motorPinAL = 1;
  int motorPinBL = 0;
  switch (direction) {
    case N:
      speedL = speed;
      speedR = speed;
      motorPinAR = 1;
      motorPinBR = 0;
      motorPinAL = 1;
      motorPinBL = 0;
      break;
    case S:
      speedL = speed;
      speedR = speed;
      motorPinAR = 0;
      motorPinBR = 1;
      motorPinAL = 0;
      motorPinBL = 1;

      break;
    case E:
      speedL = speed;
      speedR = 0;
      motorPinAR = 0;
      motorPinBR = 1;
      motorPinAL = 1;
      motorPinBL = 0;

      break;
    case W:
      speedL = 0;
      speedR = speed;
      motorPinAR = 1;
      motorPinBR = 0;
      motorPinAL = 0;
      motorPinBL = 1;

      break;
    case NW:
      speedL = speed / 1.5;
      speedR = speed;
      motorPinAR = 1;
      motorPinBR = 0;
      motorPinAL = 0;
      motorPinBL = 1;
      break;
    case SW:
      speedL = speed / 1.5;
      speedR = speed;
      motorPinAR = 1;
      motorPinBR = 0;
      motorPinAL = 0;
      motorPinBL = 1;
      break;
    case NE:
      speedL = speed;
      speedR = speed / 1.5;
      motorPinAR = 1;
      motorPinBR = 0;
      motorPinAL = 0;
      motorPinBL = 1;
      break;
    case SE:
      motorPinAR = 1;
      motorPinBR = 0;
      motorPinAL = 0;
      motorPinBL = 1;
      speedL = speed;
      speedR = speed / 1.5;
      break;
    default:
      speedL = 0;
      speedR = 0;
      motorPinAR = 1;
      motorPinBR = 0;
      motorPinAL = 0;
      motorPinBL = 1;
      break;
  }
  digitalWrite(pinDriveRightDirection1, motorPinAR);
  digitalWrite(pinDriveRightDirection2, motorPinBR);
  digitalWrite(pinDriveLeftDirection1, motorPinAL);
  digitalWrite(pinDriveLeftDirection2, motorPinBL);
  analogWrite(pinPwmDriveLeft, speedL);
  analogWrite(pinPwmDriveRight, speedR);
}

MiddlePoint setMiddlePoint(int startMiddleX, int startMiddleY) {
  MiddlePoint central;
  central.x = startMiddleX;
  central.y = startMiddleY;
  // Serial.println(central.y);

  return central;
}

Direction setMovement(int xData, int yData) {
  MiddlePoint central = setMiddlePoint(0, 0);  //random cisla tie nuly

  const int deadZoneLowX = 500 - deadZone;
  const int deadZoneHighX = 500 + deadZone;

  const int deadZoneLowY = 500 - deadZone;
  const int deadZoneHighY = 500 + deadZone;

  //Serial.println(central.y);
  if (xData > deadZoneHighX && yData > deadZoneHighY)
    return NE;

  else if (xData < deadZoneLowX && yData > deadZoneHighY)
    return NW;

  else if (xData < deadZoneHighX && xData > deadZoneLowX && yData > deadZoneHighY)
    return N;

  else if (xData < deadZoneHighX && xData > deadZoneLowX && yData < deadZoneLowY)
    return S;

  else if (xData > deadZoneHighX && yData < deadZoneHighY && yData > deadZoneLowY)
    return E;

  else if (xData < deadZoneLowX && yData < deadZoneHighY && yData > deadZoneLowY)
    return W;

  else if (xData > deadZoneHighX && yData < deadZoneLowY)
    return SE;

  else if (xData < deadZoneLowX && yData < deadZoneLowY)
    return SW;
  else
    return None;
}

int setSpeed(int joystick) {  //joystick - hodnota kt. budeme ziskavat z joysticku
  if (joystick < speedZone)   //speedzone - hranica na kt. sa bude menit rychlost, treba odmerat
    return slowSpeed;
  else
    return fastSpeed;
}
////////////Drive Manuel END ////////////////////

////////////Sonar//////////////////
void processSonar() {
  unsigned long currentMillis = millis();
  updateFishCount(currentMillis);

  if (currentMillis - previousMillisSonar >= intervalSonar) {
    previousMillisSonar = currentMillis;
    int duration = measureDistance();
    //int duration = 20;
    int distance = duration / 58;  //air
    // int distance = duration / 13.5;  //water
    // Serial.print("Distance: ");
    // Serial.print(distance);
    // Serial.print(" cm  ");
    // Serial.print("Fish Count: ");
    //   Serial.println(fishCount);
  

    if (previousDistance != -1 && previousDistance - distance > previousDistance * 0.3) {
      recordFish(currentMillis);
      // Serial.print("Fish Count: ");
      // Serial.println(fishCount);
    }
    previousDistance = distance;
  }
}

void recordFish(unsigned long timestamp) {
  for (int i = 0; i < 100; i++) {
    if (fishTimestamps[i] == 0) {
      fishTimestamps[i] = timestamp;
      fishCount++;
      break;
    }
  }
}

void updateFishCount(unsigned long currentMillis) {
  for (int i = 0; i < 100; i++) {
    if (fishTimestamps[i] != 0 && (currentMillis - fishTimestamps[i] >= 10000)) {
      fishTimestamps[i] = 0;
      if (fishCount > 0) fishCount--;
    }
  }
}

int measureDistance() {
  digitalWrite(TRIGPIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGPIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGPIN, LOW);
  return pulseIn(ECHOPIN, HIGH);
  //return 20;
}

int measureDistanceNonBlocking() {
  if (!echoReceived) {
    if (triggerTime == 0) {
      // Trigger the pulse
      digitalWrite(TRIGPIN, LOW);
      delayMicroseconds(2);
      digitalWrite(TRIGPIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIGPIN, LOW);
      triggerTime = micros();
    } else if (micros() - triggerTime > 20000) { // 20ms timeout
      triggerTime = 0; // Reset for the next trigger
      return -1; // Timeout
    }
  } else {
    // Calculate pulse duration
    unsigned long duration = echoEndTime - echoStartTime;
    echoReceived = false; // Reset for the next measurement
    triggerTime = 0; // Reset for the next trigger
    return duration;
  }
  return -1; // Measurement not complete yet
}

void echoISR() {
  if (digitalRead(ECHOPIN) == HIGH) {
    echoStartTime = micros();
  } else {
    echoEndTime = micros();
    echoReceived = true;
  }
}

/////////////Sonar End//////////////


/////////////////GPS////////////////
void processGps() {
  //Serial.println("HELLO");
  while (Serial7.available() > 0) {
   // Serial.println("GPS AVAILABLE");
    char c = Serial7.read();
    //Serial.print(c);
    if(gps.encode(c)){
      if(gps.location.isValid()){
        unsigned long currentMillis = millis();

        if(currentMillis - gps_previousMillis > gps_interval){
          if(gps_point_counter < 4){
            getGPSPoint();
            if(gps_dataAvailable == 1){
              calculateGPSDistanceAndHeading();
            }
          }
          gps_previousMillis = currentMillis;
        }
        
        if(gps_point_counter >= 4){
          gps_point_counter--;
          for(uint8_t i = 0; i < gps_point_counter; i++){
            gps_point_lat_lon_array[i][0] = gps_point_lat_lon_array[i+1][0];
            gps_point_lat_lon_array[i][1] = gps_point_lat_lon_array[i+1][1];
          }
        }

        currentMillis = millis();
        if(currentMillis - gps_previousMillis2 > 5000 && gps_hasCorrectData){
          Serial.println("We here");
          dataToSend.actualGpsPositionLat1 = 0;
          dataToSend.actualGpsPositionLon1 = 0;
          dataToSend.actualGpsPositionLat2 = 0;
          dataToSend.actualGpsPositionLon2 = 0;
          dataToSend.NumOfSats = 0;
          gps_hasCorrectData = 0;
      }
    }
  }
  }
  // if(!Serial7.available()){
  //   Serial.println("GPS UNAVAILABLE");
  //   gps_dataAvailable = 0;
  // }

  if(gps_hasCorrectData){
    Serial.println();
    Serial.print("Coordinates = ");
    double temp = getNewestLat();
    Serial.print(temp,10);
    Serial.print(", ");
    //dataToSend.actualGpsPositionLat = temp;
    temp = getNewestLng();
    Serial.print(temp,10);
    Serial.print(", ");
    //dataToSend.actualGpsPositionLon = temp;
    dataToSend.NumOfSats = gps_satelite_number;
    Serial.print(", ");
    Serial.println(gps_satelite_number);

    uint temp1 = round(getNewestLat() * 10000000);
    uint temp2 = round(getNewestLng() *10000000);
    dataToSend.actualGpsPositionLat1 = round(temp1 / 100000);
    dataToSend.actualGpsPositionLat2 = round(temp1 % 100000);
    dataToSend.actualGpsPositionLon1 = round(temp2 / 100000);
    dataToSend.actualGpsPositionLon2 = round(temp2 % 100000);
    //gps_dataAvailable = 0;
  }
}

double getNewestLat(){
  if(gps_point_counter < 1 || gps_point_counter > 4){
    return -1.0;
  }
  return gps_point_lat_lon_array[gps_point_counter][0];
}

double getNewestLng(){
  if(gps_point_counter < 1 || gps_point_counter > 4){
    return -1.0;
  }
  return gps_point_lat_lon_array[gps_point_counter][1];
}

double haversine(double lat1, double lon1, double lat2, double lon2) {
    const double rEarth2 = 6372795.0;
    const double rEarth = 6371000.0;
    double x = pow( sin( ((lat2 - lat1)*M_PI/180.0) / 2.0), 2.0 );
    double y = cos(lat1*M_PI/180.0) * cos(lat2*M_PI/180.0);
    double z = pow( sin( ((lon2 - lon1)*M_PI/180.0) / 2.0), 2.0 );
    double a = x + y * z;
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0-a));
    double d = rEarth2 * c;
    return d;
}

void getGPSPoint(){
  if(gps_point_counter < 4){
    if (gps.location.isValid() && gps.location.isUpdated()) {
      if(gps_point_counter > 1){
        unsigned long currentMillis = millis();
        gps_dataAvailable = 1;
        gps_hasCorrectData = 1;
        gps_previousMillis2 = currentMillis;
      }
      else
        gps_dataAvailable = 0;

      Serial.print("1");
      Serial.print(",");
      
      gps_satelite_number = gps.satellites.value();
      Serial.print(gps_satelite_number);
      Serial.print(",");

      gps_point_lat_lon_array[gps_point_counter][0] = gps.location.lat();
      gps_point_lat_lon_array[gps_point_counter][1] = gps.location.lng();

      Serial.print(gps_point_lat_lon_array[gps_point_counter][0],10);
      Serial.print(",");

      Serial.print(gps_point_lat_lon_array[gps_point_counter][1],10);
      Serial.print(",");
      gps_point_counter++;
    }
    else{
      gps_dataAvailable = 0;
    }
  }
}

void calculateGPSDistanceAndHeading(){
  double distance = haversine(gps_point_lat_lon_array[gps_point_counter-2][0] , gps_point_lat_lon_array[gps_point_counter-2][1], 
                             gps_point_lat_lon_array[gps_point_counter-1][0], gps_point_lat_lon_array[gps_point_counter-1][1]);
  if(distance > 2.0){
    distance = 0.0;
  }
  if(distance != 0.0){
    fullDistanceTraveled += distance;
  }

  gps_heading_degrees = gps.courseTo(gps_point_lat_lon_array[gps_point_counter-2][0] , gps_point_lat_lon_array[gps_point_counter-2][1], 
                              gps_point_lat_lon_array[gps_point_counter-1][0], gps_point_lat_lon_array[gps_point_counter-1][1]);
  // Serial.print(distance,5);
  // Serial.print(",");
  // Serial.print(fullDistanceTraveled,5);
  // Serial.print(",");
  // Serial.println(gps_heading_degrees);
}
////////////////GPS END////////////

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