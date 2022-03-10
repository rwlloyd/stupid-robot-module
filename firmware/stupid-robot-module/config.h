#ifndef CONFIG_H
#define CONFIG_H
#include <Arduino.h>

#define errorPin 12
// Communication 
#define MESSAGELENGTH 5
#define BAUDRATE 115200

// Big Servo
#define enA 11          //Enable pin to motor driver
#define in1 9           //L298N in 1
#define in2 10          //L298N in 2
#define hallS 4         // Inverted Hall Sensor Input
#define potPin A0       // Analog input for adjustment
#define buttonPin 11    // Button Pin 

#define ENCODER_NUM_LINES 1000
#define ENCODER_PULLEY_RATIO 2.5
#define encoder0PinA 2  // Encoder 0 pin A
#define encoder0PinB 3  // Encoder 0 pin B

#define SERVO_HOME_OFFSET 0;

// Servo PID Gains
#define SERVO_P 10
#define SERVO_I 0.1
#define SERVO_D 0



// Wheel

#define WHEEL_DIA_MM 165

#define WHEEL_EN_PIN 8
#define WHEEL_SIGNAL_PIN 7
#define WHEEL_DIRECTION_PIN 6
#define WHEEL_SPEED_PIN 5

// Motor Driver deadzon (+/-)
#define WHEEL_SPEED_THRESHOLD 15
// Wheel Velocity PID Gains
#define WHEEL_VEL_P 3
#define WHEEL_VEL_I 0.1
#define WHEEL_VEL_D 0

// Wheel Position PID Gains
#define WHEEL_POS_P 1
#define WHEEL_POS_I 0
#define WHEEL_POS_D 0

// Wheel Hall Sensor Pins
#define WHEEL_HALL_FB_A A1
#define WHEEL_HALL_FB_B A2
#define WHEEL_HALL_FB_C A3

#define WHEEL_NUM_POLES 45

#endif
