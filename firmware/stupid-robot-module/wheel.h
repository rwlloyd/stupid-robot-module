#ifndef WHEEL_H
#define WHEEL_H
#include <Arduino.h>

extern int wheelSetpoint;
extern int wheelDirection;
extern int wheelMode;

void startWheel();
void doWheelControl();
void doBasicControl();
void doVelocityControl();
void doPositionControl();
void wheelStop();
void wheelMove(int s_new, bool dir_new);
void calculateRealSpeed();
void displayMotorSignals();
void displayWheelSpeeds();
void displayWheelPositions();
void odom();
void doA();
void doB();
void doC();

#endif
