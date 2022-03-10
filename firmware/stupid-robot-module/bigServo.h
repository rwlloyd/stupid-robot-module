#ifndef BIGSERVO_H
#define BIGSERVO_H
#include <Arduino.h>

extern int desiredAngle;

void startServo();
void goClock(int s);
void goAntiClock(int s);
void doServoHoming();
float stepsToAngle(int steps);
float angleToSteps(float angle);
void gotoPosition(int p);
void gotoAngle(int a);
void doServoControl();
void doEncoderA();
void doEncoderB();

#endif
