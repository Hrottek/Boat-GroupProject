// 16/3/2024

#ifndef RIADENIE_H
#define RIADENIE_H

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
    S
} Direction; //svetove strany

MiddlePoint setMiddlePoint(int startMiddleX, int startMiddleY);

Direction setMovement(int xData, int yData);

int setSpeed(int joystick);

#endif // !RIADENIE_H