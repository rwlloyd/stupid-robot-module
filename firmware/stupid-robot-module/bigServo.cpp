#include "config.h"
#include "bigServo.h"
#include "Arduino.h"
#include "PinChangeInterrupt.h"
#include "ucontrol.h"

#include "PID_v1.h"
#include "avr/interrupt.h"

bool debugServo = false;
int desiredAngle = 0;

int rotDirection = 0;
int pressed = false;

double encoder0Pos = 0;
unsigned int tmp_Pos = 1;

// Variable for pulses per rotation of incremental encoder
int steps = ENCODER_NUM_LINES;
// Variable for encoder pulley:shaft pulley ratio. eg. 40:16 = 2.5
float ratio = ENCODER_PULLEY_RATIO;
// calculate encoder steps for one shaft rotation
float shaftSteps = steps * ratio;
float resolution = 360 / shaftSteps;

int currentPoint = 0;
boolean A_set;
boolean B_set;

bool homed = false;
int homeStatus = 0;
bool magnetFound = false;
bool magnetSize = 0;
bool detected = false;
int magnetMax = 0;
int magnetMin = 0;

//PID STUFF

//Define Variables we'll be connecting to
double serSetpoint, serInput, serOutput, serOutputA;

//Specify the links and initial tuning parameters
double sKp = SERVO_P, sKi = SERVO_I, sKd = SERVO_D;
PID motorPID(&serInput, &serOutput, &serSetpoint, sKp, sKi, sKd, DIRECT);

unsigned long currentMillis;
long previousMillis = 0;    // set up timers
long interval = 40;        // time constant for timers

// Interrupt on A changing state
void doEncoderA() {
  // Low to High transition?
  if (digitalRead(encoder0PinA) == HIGH) {
    A_set = true;
    if (!B_set) {
      encoder0Pos++;
    }
  }
  // High-to-low transition?
  if (digitalRead(encoder0PinA) == LOW) {
    A_set = false;
  }
}


// Interrupt on B changing state
void doEncoderB() {
  // Low-to-high transition?
  if (digitalRead(encoder0PinB) == HIGH) {
    B_set = true;
    if (!A_set) {
      encoder0Pos--;
    }
  }
  // High-to-low transition?
  if (digitalRead(encoder0PinB) == LOW) {
    B_set = false;
  }
}

void doHall() {
  // Read the hall sensor. its pin is pulled up, so true is low.
  // detection of magnet
  if (digitalRead(hallS) == LOW) {
    detected = true;
  }
  if (digitalRead(hallS) == HIGH) {
    detected = false;
  }
}

void startServo() {

  setServoPWM();

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(hallS, INPUT_PULLUP);
  pinMode(potPin, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  // Set initial rotation direction
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);

  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(0, doEncoderA, CHANGE);
  // encoder pin on interrupt 1 (pin 3)
  attachInterrupt(1, doEncoderB, CHANGE);

  attachPCINT(digitalPinToPCINT(hallS), doHall, CHANGE);

  int deadZone = 10;

  //turn the PID on
  motorPID.SetMode(AUTOMATIC);
  motorPID.SetOutputLimits(-255, 255);
  motorPID.SetSampleTime(20);

  Serial.begin(BAUDRATE);
  if (debugServo) {
    Serial.print("Calculated Steps per rotation: "); Serial.println(shaftSteps);
    Serial.print("Calculated angular resolution: "); Serial.print(resolution); Serial.println(" degs per encoder tick");
  }
}

void goClock(int s) {
  analogWrite(in1, 0);
  analogWrite(in2, s);
}

void goAntiClock(int s) {
  analogWrite(in1, s);
  analogWrite(in2, 0);
}

bool seekedClock = false;

void doServoHoming() {
  if (!homed && !detected) {
    if (encoder0Pos > (shaftSteps * 0.2)) {
      seekedClock = true;
    }
    if (!seekedClock) {
      goClock(127);
    } else if (seekedClock) {
      goAntiClock(127);
    }
    homeStatus = 1;
  }
  if (homeStatus == 1 && detected) {
    encoder0Pos = SERVO_HOME_OFFSET;
    homed = true;
  }
}

float stepsToAngle(int steps) {
  return float(steps * resolution);
}

float angleToSteps(float angle) {
  return (angle / resolution);
}

float currentAngle() {
  return stepsToAngle(encoder0Pos);
}

void gotoPosition(int p) {
  serSetpoint = p;
}

void gotoAngle(int a) {
  serSetpoint = angleToSteps(a);
}

void doServoControl() {
  if (!homed) {
    doServoHoming();
  }
  currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {  //start timed event
    previousMillis = currentMillis;

    motorPID.Compute();

//    Setpoint = map(analogRead(A0), 0, 1024, 0, 255); // only here for testing
//    Input = map(encoder0Pos, 0, shaftSteps, 0, 255);

    serSetpoint = angleToSteps(map(desiredAngle, 0, 255, 0, 360));

    serInput = encoder0Pos;

    if (serOutput > 0) {
      analogWrite(in1, serOutput);
      analogWrite(in2, 0);
    }
    else if (serOutput < 0) {
      serOutputA = abs(serOutput);
      analogWrite(in1, 0);
      analogWrite(in2, serOutputA);
    }

    // Run the motor by enabling the driver with a PWM signal
    //    analogWrite(enA, Output); // Send PWM signal to L298N Enable pin
    analogWrite(enA, HIGH);
  }
}
