/*
   This file structure is an attampt to modularize what are becoming uncomfortably lengthy piles of code.
   https://forum.arduino.cc/t/how-to-properly-include-functions-written-on-other-sketch-tabs/565672/4#msg4005111

*/
#include "safety.h"
#include "comms.h"
#include "bigServo.h"
#include "wheel.h"

void setup() {
  startSafety();
  startComms();
  startServo();
  startWheel();
  doServoHoming();
}

void loop() {
  checkComms();
  if (!error) {
    doWheelControl(); // Modes: 0: basic, 1:velocity, 2:position
    doServoControl();
  }
}

/*

   Simple Processing sketch to control module

  import processing.serial.*;
  Serial myPort;

  byte[] inBuffer = new byte[3];
  int[] myString;

  boolean error = false;


  boolean contServo = false;
  boolean contWheel = false;

  int servoAngle = 0; // 0-255
  int wheelSpeed  = 0;

  void setup() {
  frameRate(30);
  size(500, 500);
  background(51);
  printArray(Serial.list());

  myPort = new Serial(this, Serial.list()[1], 115200);
  myPort.clear();
  sendCommand(false, 0, 0);
  }

  void draw() {
  background(51);
  while (myPort.available() > 0) {
    inBuffer = myPort.readBytes();
    myPort.readBytes(inBuffer);
    if (inBuffer != null) {
      myString = int(inBuffer);
      println(myString);
    }
  }
  stroke(255);
  // Draw neutral Line
  line(0, height/2, width, height/2);
  // Display enabled status
  if (error) {
    text("Motors Enabled", 20, height - 20);
  } else {
    text("Motors Disabled", 20, height - 20);
  }
  //if (contTool) {
  //  text("Tool Control Enabled", 20, height - 40);
  //} else {
  //  text("Tool Control Disabled", 20, height - 40);
  //}
  //if (contSteer) {
  //  text("Steering Control Enabled", 20, height - 60);
  //} else {
  //  text("Steering Control Disabled", 20, height - 60);
  //}
  //// Display tool height
  //text("Tool Height " + floor(map(tool_height, 0, 255, 0, 100)) + "%", 200, height - 20);
  //// Display steering Status

  //text("Steer " + steer, 300, height - 20);

  //if (!contTool) {
  //  tool_height = last_tool_height;
  //}
  //if(!contSteer){
  //  steer = 127;
  //}


  sendCommand(error, servoAngle, wheelSpeed);
  }

  void mousePressed() {
  wheelSpeed = abs(int(map(mouseY, 0, height, 0, 255)));
  servoAngle = abs(int(map(mouseX, 0, width, 0, 255)));
  }

  void sendCommand(boolean error_in, int servo_angle_in, int wheel_speed_in) {
  myPort.write(byte(error_in));
  myPort.write(byte(servo_angle_in));
  myPort.write(byte(wheel_speed_in));
  }

  void keyPressed() {
  // 'e' = toggle enable
  if (key == 'e' || key == 'E') {
    error =! error;
  }
  }
*/
