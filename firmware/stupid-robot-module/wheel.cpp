#include "safety.h"
#include "config.h"
#include "wheel.h"
#include "arduino.h"
#include "PinChangeInterrupt.h"
#include "PID_v1.h"

// Generally useful information about brushless motors
// https://www.nxp.com/docs/en/application-note/AN10661.pdf

int wheel_EN_pin = WHEEL_EN_PIN;  //EL
int wheel_Signal_hall_pin = WHEEL_SIGNAL_PIN;   // Signal - Hall sensor <-- This can be used as a speedo/odom signal
int wheel_ZF_Direction_pin = WHEEL_DIRECTION_PIN;  // ZF
int wheel_VR_speed_pin = WHEEL_SPEED_PIN;    //VR

// inputs
int wheelSetpoint = 0;
int wheelDirection = false;
int wheelMode = 0;

bool debug = false;

// globals
int wheelSpeed = 0;
float wheelRealSpeed = 0; // speed in rpm
int wheelSignalPos = 0;
// Variables to help filtering the velocity calculations
float velocityFiltered = 0;
float velocityFilteredPrev = 0;

// 0-255 signal used when in position control
float power = 0;

long prevT = 0;
int posPrev = 0;
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;
// Variable to keep track of the BLDC Phases and direction
volatile boolean encAVal = 0;
volatile bool encBVal = 0;
volatile bool encCVal = 0;
volatile bool encDir = 0;

//PID STUFF

//Define Variables we'll be connecting to
double velSetpoint, velInput, velOutput, velOutputA;

//Specify the links and initial tuning parameters
double vKp = WHEEL_VEL_P, vKi = WHEEL_VEL_I, vKd = WHEEL_VEL_D;
PID velPID(&velInput, &velOutput, &velSetpoint, vKp, vKi, vKd, DIRECT);

//Define Variables we'll be connecting to
double posSetpoint, posInput, posOutput, posOutputA;

//Specify the links and initial tuning parameters
double pKp = WHEEL_POS_P, pKi = WHEEL_POS_I, pKd = WHEEL_POS_D;
PID posPID(&posInput, &posOutput, &posSetpoint, pKp, pKi, pKd, DIRECT);

unsigned long currentWheelMillis; // for pid
long previousWheelMillis = 0;    // set up timers for pid
long wheelInterval = 35;        // time constant for pid timers

long previousSpeedMillis = 0;
unsigned long currentSpeedMillis;
long speedInterval;
int current_wheel_speed = WHEEL_NUM_POLES / speedInterval;

void startWheel() {
  Serial.begin(115200);
  pinMode(potPin, INPUT);
  pinMode(wheel_EN_pin , OUTPUT);           //stop/start - EL
  pinMode(wheel_Signal_hall_pin , INPUT);   //plus       - Signal
  pinMode(wheel_ZF_Direction_pin , OUTPUT); //direction  - ZF

  //Hall sensor detection - Count stps
  //attachInterrupt(digitalPinToInterrupt(wheel_Signal_hall_pin), odom, CHANGE);
  //https://github.com/NicoHood/PinChangeInterrupt/blob/master/examples/PinChangeInterrupt_Led/PinChangeInterrupt_Led.ino
  attachPCINT(digitalPinToPCINT(wheel_Signal_hall_pin), odom, CHANGE);

  pinMode(WHEEL_HALL_FB_A, INPUT);
  pinMode(WHEEL_HALL_FB_B, INPUT);
  pinMode(WHEEL_HALL_FB_C, INPUT);
  attachPCINT(digitalPinToPCINT(WHEEL_HALL_FB_A), doA, CHANGE);
  attachPCINT(digitalPinToPCINT(WHEEL_HALL_FB_B), doB, CHANGE);
  attachPCINT(digitalPinToPCINT(WHEEL_HALL_FB_C), doC, CHANGE);

  //turn the PID on
  velPID.SetMode(AUTOMATIC);
  velPID.SetOutputLimits(-255, 255);
  velPID.SetSampleTime(20);

  //turn the PID on
  posPID.SetMode(AUTOMATIC);
  posPID.SetOutputLimits(-255, 255);
  posPID.SetSampleTime(10);

}

void doWheelControl() {
  velPID.Compute();
  posPID.Compute();
  //displayMotorSignals();

  // 30ms loop
  currentWheelMillis = millis();
  if (currentWheelMillis - previousWheelMillis >= wheelInterval) {  //start timed event
    previousWheelMillis = currentWheelMillis;

    calculateRealSpeed();


    if (wheelMode == 0) {
      doBasicControl();
    } else if (wheelMode == 1) {
      doVelocityControl();
    } else if (wheelMode == 2) {
      doPositionControl();
    }
  }
}

void doBasicControl() {
  //int BasicSetpoint = floor(map(analogRead(potPin), 0, 1024, -254, 254));
  int basicSetpoint = wheelSetpoint;
  if (!wheelDirection){
    basicSetpoint = -1* basicSetpoint;
  }
  if (basicSetpoint == 0) {
    wheelStop();
  }
  else if (basicSetpoint > 0) {
    wheelMove(abs(basicSetpoint), wheelDirection);
  }
  else if (basicSetpoint < 0) {
    wheelMove(abs(basicSetpoint), wheelDirection);
  }
  if (debug) {
    Serial.print(basicSetpoint);
    Serial.print(",");
    Serial.println(wheelSignalPos);
  }
}

void doVelocityControl() {
  //https://github.com/curiores/ArduinoTutorials/blob/main/SpeedControl/SpeedControl_NoAtomic/SpeedControl_NoAtomic.ino

  if (debug) {
    displayWheelSpeeds();
  }

  //velSetpoint = map(analogRead(A0), 0, 1024, -25, 25);
  velSetpoint = wheelSetpoint / 10;
  if (!wheelDirection){
    velSetpoint = -1* velSetpoint;
  }
  velInput = velocityFiltered;
  if (velOutput > 0) {
    wheelMove(constrain(abs(velOutput), 0, 254), wheelDirection);
  }
  else if (velOutput < 0) {
    wheelMove(constrain(abs(velOutput), 0, 254), wheelDirection);
  }
}

void doPositionControl() {
  // https://github.com/curiores/ArduinoTutorials/blob/main/encoderControl/part4_NoAtomic/part4_NoAtomic.ino

  if (debug) {
    displayWheelPositions();
  }

  //posSetpoint = map(analogRead(A0), 0, 1024, -300, 300);
  posSetpoint = wheelSetpoint;
  if (!wheelDirection){
    posSetpoint = -1* posSetpoint;
  }
  posInput = wheelSignalPos;

  power = fabs(posOutput);
  if (power > 255) {
    power = 255;
  }
  if (posOutput > 0) {
    wheelMove(power, true);
  }
  else if (posOutput < 0) {
    wheelMove(power, false);
  }
}

void wheelStop() {
  digitalWrite(wheel_EN_pin , LOW);
}

void wheelMove(int s_new, bool dir_new) {
  int s = floor(abs(map(s_new, 0, 255, WHEEL_SPEED_THRESHOLD, 255)));
  analogWrite(wheel_VR_speed_pin, s_new);
  //digitalWrite(wheel_EN_pin , LOW);
  digitalWrite(wheel_ZF_Direction_pin , dir_new);
  digitalWrite(wheel_EN_pin, HIGH);
}

void displayMotorSignals() {
  Serial.print(encDir + 2);
  Serial.print(",");
  Serial.print(digitalRead(WHEEL_HALL_FB_A));
  Serial.print(",");
  Serial.print(digitalRead(WHEEL_HALL_FB_B) - 2);
  Serial.print(",");
  Serial.println(digitalRead(WHEEL_HALL_FB_C) - 4);
}

void calculateRealSpeed() {
  //https://github.com/curiores/ArduinoTutorials/blob/main/SpeedControl/SpeedControl_NoAtomic/SpeedControl_NoAtomic.ino
  // read the position and velocity
  int pos = 0;
  float velocity = 0;
  noInterrupts(); // disable interrupts temporarily while reading
  pos = wheelSignalPos;
  velocity = velocity_i;
  interrupts(); // turn interrupts back on

  // Compute velocity with by counting Signal ticks between loops
  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / 1.0e6;
  velocity = (pos - posPrev) / deltaT;
  posPrev = pos;
  prevT = currT;

  // Convert count/s to RPM
  wheelRealSpeed = velocity / 600.0 * 60.0;
  // Low-pass filter (25 Hz cutoff)
  // Calculates the filtered angular velocity of wheel in rpm
  velocityFiltered = 0.854 * velocityFiltered + 0.0728 * wheelRealSpeed + 0.0728 * velocityFilteredPrev;
  velocityFilteredPrev = wheelRealSpeed;
}

void displayWheelSpeeds() {
  Serial.print(wheelRealSpeed);
  Serial.print(",");
  Serial.print(velocityFiltered);
  Serial.print(",");
  Serial.println(velOutput);
}

void displayWheelPositions() {
  Serial.print(posSetpoint);
  Serial.print(",");
  Serial.print(wheelSignalPos);
  Serial.print(",");
  Serial.println(power); //posOutput);
}

// function to count SignalPulses from driver feedback
// uses the direction found from looking at feedback from the hall sensors in the motor
// also computes velocity of the wheel rotation
void odom() {
  if (encDir) {
    wheelSignalPos++;
  }
  if (!encDir) {
    wheelSignalPos--;
  }
  // Compute and save odometry variables for this loop
  long currT = micros();
  float deltaT = ((float) (currT - prevT_i)) / 1.0e6;
  velocity_i = 1 / deltaT;
  prevT_i = currT;
}

// interrupt functions to count steps from hall sensors and
// figure out direction of rotation (encDir)
void doA() {
  if (encAVal) {
    encAVal = false;
  } else {
    encAVal = true;
    // We can Also tell the direction from the state of sensor C
    if (encCVal) {
      encDir = true;
    }
  }
}
void doB() {
  if (encBVal) {
    encBVal = false;
  } else {
    encBVal = true;
  }
}
void doC() {
  if (encCVal) {
    encCVal = false;
  } else {
    encCVal = true;
    if (encAVal) {
      encDir = false;
    }
  }
}
