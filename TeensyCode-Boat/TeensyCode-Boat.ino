#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// 16/3/2024

#define deadZone 20
#define speedZone 20

#define slowSpeed 200 // PWM
#define fastSpeed 255

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
} Direction; //svetove strany


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

SensorData dataToSend;
commandData dataToReceive;

unsigned long previousMillisLeft = 0; // Stores last time the functions were triggered
const long intervalLeft = 1000; // Interval at which to run the second function (milliseconds)
unsigned long previousMillisRight = 0; // Stores last time the functions were triggered
const long intervalRight = 1000; // Interval at which to run the second function (milliseconds)
bool isTimerLeftDumpTriggered = false;
bool isTimerRightDumpTriggered = false;

// Pin setup for driving and dumping
const int pinDriveLeftDirection1 = 6;
const int pinDriveLeftDirection2 = 7;
const int pinDriveRightDirection1 = 8;
const int pinDriveRightDirection2 = 9;
const int pinDumpLeftDirection1 = 23;
const int pinDumpLeftDirection2 = 22;
const int pinDumpRightDirection1 = 21;
const int pinDumpRightDirection2 = 20;
const int pinPwmDriveLeft = 2; // PWM control for left drive
const int pinPwmDriveRight = 3; // PWM control for right drive
const int pinPwmDumpLeft = 4; // PWM control for left dump
const int pinPwmDumpRight = 5; // PWM control for right dump
const int pwmValueDumpLeft = 128;   // Set a PWM value between 0 (0% duty cycle) and 255 (100% duty cycle) //TODO zistit
const int pwmValueDumpRight = 128;   // Set a PWM value between 0 (0% duty cycle) and 255 (100% duty cycle)

//pins for NRF24L01
const int pinCE = 19;
const int pinCSN = 10;
RF24 radio(pinCE, pinCSN); // Adjust pin numbers for CE, CSN to match your Teensy setup
const byte addresses[][6] = {"1Node", "2Node"};


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

}

void loop() {
  if (radio.available()) {
    radio.read(&dataToReceive, sizeof(commandData));
    Serial.print(dataToReceive.yAxis);
    Serial.print("  ");
    Serial.print(dataToReceive.xAxis);
    Serial.print("  ");
    Serial.print(isTimerLeftDumpTriggered);
    Serial.print("  ");
    Serial.println(isTimerRightDumpTriggered);

    if(dataToReceive.leftDump || isTimerLeftDumpTriggered){ // TODO If lost connection while dumping This makes Dump Motor Left to dump
      triggerFunctionsNonBlockingLeftDump();
    }

    if(dataToReceive.rightDump || isTimerRightDumpTriggered){ // TODO If lost connection while dumping This makes Dump Motor Right to dump
      triggerFunctionsNonBlockingRightDump();
    }

    drive(dataToReceive.xAxis,dataToReceive.yAxis);

    sendDataToArduino();
  }
  else{
    //Serial.println("Radio Didnt Receive"); //TODO if radio didnt receive get time and if not for 15 sec then go to home.
  }
  delay(1);
}

void triggerFunctionsNonBlockingLeftDump() {
  unsigned long currentMillis = millis();
  
  if (!isTimerLeftDumpTriggered) {
    previousMillisLeft = currentMillis; // Save the time of the trigger
    isTimerLeftDumpTriggered = true;
    analogWrite(pinPwmDumpLeft, pwmValueDumpLeft);
    digitalWrite(pinDumpLeftDirection1, 1);
    digitalWrite(pinDumpLeftDirection2, 0);
  }

  if (isTimerLeftDumpTriggered && currentMillis - previousMillisLeft >= intervalLeft) {
    isTimerLeftDumpTriggered = false; // Reset the trigger flag
    analogWrite(pinPwmDumpLeft, 0);
    digitalWrite(pinDumpLeftDirection1, 0);
    digitalWrite(pinDumpLeftDirection2, 0);
  }
}

void triggerFunctionsNonBlockingRightDump() {
  unsigned long currentMillis = millis();
  
  if (!isTimerRightDumpTriggered) {
    previousMillisRight = currentMillis; // Save the time of the trigger
    isTimerRightDumpTriggered = true;
    analogWrite(pinPwmDumpRight, pwmValueDumpRight);
    digitalWrite(pinDumpRightDirection1, 1);
    digitalWrite(pinDumpRightDirection2, 0);
  }

  if (isTimerRightDumpTriggered && currentMillis - previousMillisRight >= intervalRight) {
    isTimerRightDumpTriggered = false; // Reset the trigger flag
    analogWrite(pinPwmDumpRight, 0);
    digitalWrite(pinDumpRightDirection1, 0);
    digitalWrite(pinDumpRightDirection2, 0);
  }
}
void sendDataToArduino() {
  radio.stopListening();
  dataToSend.sonarDistance = 5; // Prepare your data //TODO sonar, prejdena vzdialenost, gps data
  while(radio.write(&dataToSend, sizeof(SensorData))){
    //Keep sending until you acnowledge 
    //TODO after 15 sec return home
  }
  radio.startListening();
}

void drive(int xData, int yData){
    setMiddlePoint(500, 500);
    Direction direction = setMovement(xData, yData);
    Serial.println(direction);
    int speed = setSpeed(xData);
    static int speedR;
    static int speedL;
    int motorPinAR=1;
    int motorPinBR=0;
    int motorPinAL=1;
    int motorPinBL=0;
    switch (direction) {
        case N:
            speedL=speed;
            speedR=speed;
            motorPinAR=1;
            motorPinBR=0;
            motorPinAL=1;
            motorPinBL=0;
            break;
        case S:
            speedL=speed;
            speedR=speed;
            motorPinAR=0;
            motorPinBR=1;
            motorPinAL=0;
            motorPinBL=1;

            break;
        case E:
            speedL=speed;
            speedR=0;
            motorPinAR=0;
            motorPinBR=1;
            motorPinAL=1;
            motorPinBL=0;

            break;
        case W:
            speedL=0;
            speedR=speed;
            motorPinAR=1;
            motorPinBR=0;
            motorPinAL=0;
            motorPinBL=1;

            break;
        case NW:
            speedL=speed/1.5;
            speedR=speed;
            motorPinAR=1;
            motorPinBR=0;
            motorPinAL=0;
            motorPinBL=1;
            break;
        case SW:
            speedL=speed/1.5;
            speedR=speed;
            motorPinAR=1;
            motorPinBR=0;
            motorPinAL=0;
            motorPinBL=1;
            break;
        case NE:
            speedL=speed;
            speedR=speed/1.5;
            motorPinAR=1;
            motorPinBR=0;
            motorPinAL=0;
            motorPinBL=1;
            break;
        case SE:
            motorPinAR=1;
            motorPinBR=0;
            motorPinAL=0;
            motorPinBL=1;
            speedL=speed;
            speedR=speed/1.5;
            break;
        default:
            speedL=0;
            speedR=0;
            motorPinAR=1;
            motorPinBR=0;
            motorPinAL=0;
            motorPinBL=1;
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
    Serial.println(central.y);

    return central;
}

Direction setMovement(int xData, int yData) {
    MiddlePoint central = setMiddlePoint(0, 0); //random cisla tie nuly

    const int deadZoneLowX = 500 - deadZone;
    const int deadZoneHighX = 500 + deadZone;

    const int deadZoneLowY = 500 - deadZone;
    const int deadZoneHighY = 500 + deadZone;

    Serial.println(central.y);
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
    if (joystick < speedZone) //speedzone - hranica na kt. sa bude menit rychlost, treba odmerat
        return slowSpeed;
    else
        return fastSpeed;
}


