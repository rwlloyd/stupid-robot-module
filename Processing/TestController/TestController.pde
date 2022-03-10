import processing.serial.*;
Serial myPort;

byte[] inBuffer = new byte[3];
int[] myString;

boolean error = false;
boolean servoEnable = false;
boolean wheelEnable = false;

boolean basicControl = false;
boolean velocityControl = false;
boolean positionControl = false;

boolean contServo = false;
boolean contWheel = false;

int servoAngle = 0; // 0-255
int Setpoint = 0;
boolean Direction = true;
int mode = 0;

void setup() {
  frameRate(30);
  size(500, 600);
  background(51);
  printArray(Serial.list());

  myPort = new Serial(this, Serial.list()[1], 115200);
  myPort.clear();
  sendCommand(error, servoAngle, Setpoint, Direction, mode);
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
  // Draw neutral Lines
  line(0, height/2, width, height/2);
  line(width/2, 0, width/2, height);
  // Display enabled status
  if (error) {
    text("Enabled. 'e' to enable.", 20, height - 20);
  } else {
    text("Press 'e' to ESTOP", 20, height - 20);
  }
  // Display Current control mode
  if (!basicControl && !velocityControl && !positionControl) {
    text("Select Mode: b. Basic(0), v. Velocity(1), p. Position(2)", 20, height-40);
  }
  if (!wheelEnable) {
    text("'w' to enable wheel", width - 125, height-40);
  }
  if (!servoEnable) {
    text("'s' to enable servo", width - 125, height-20);
  }
  if (basicControl) {
    text("Mode: 0. Basic", 20, height-40);
    text(Setpoint, width - 50, height-40);
  }
  if (velocityControl) {
    text("Mode: 1. Velocity", 20, height-40);
    text(Setpoint, width - 50, height-40);
  }
  if (positionControl) {
    text("Mode: 2. Position", 20, height-40);
    text(Setpoint, width - 50, height-40);
  }
  if (!servoEnable) {
    servoAngle = 0;
  }
  if (!wheelEnable) {
    Setpoint = 0;
  }



  sendCommand(error, servoAngle, Setpoint, Direction, mode);
  delay(50);
}

void mousePressed() {
  Setpoint = int(map(mouseY, height, 0, -255, 255));
  if (Setpoint>0) {
    Direction = true;
  } else {
    Direction = false;
  }
  
  servoAngle = abs(int(map(mouseX, 0, width, 0, 255)));
}

void sendCommand(boolean error_in, int servo_angle_in, int Setpoint, boolean Direction, int mode) {
  myPort.write(byte(error_in));
  myPort.write(byte(servo_angle_in));
  myPort.write(byte(abs(Setpoint)));
  myPort.write(byte(Direction));
  myPort.write(byte(mode));
}

void keyPressed() {
  // 'e' = toggle enable 
  if (key == 'e' || key == 'E') {
    error =! error;
  }
  // 'b' = toggle basic control  
  if (key == 'b' || key == 'B') {
    Setpoint = 0;
    mode = 0;
    basicControl = true;
    velocityControl = false;
    positionControl = false;
  }
  // 'v' = toggle basic control  
  if (key == 'v' || key == 'V') {
    Setpoint = 0;
    mode = 1;
    basicControl = false;
    velocityControl = true;
    positionControl = false;
  }
  // 'p' = toggle basic control  
  if (key == 'p' || key == 'P') {
    Setpoint = 0;
    mode = 2;
    basicControl = false;
    velocityControl = false;
    positionControl = true;
  }
  if (key == 's' || key == 'S') {
    servoEnable = !servoEnable;
  }
  if (key == 'w' || key == 'W') {
    wheelEnable = !wheelEnable;
  }
}
