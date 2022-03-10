#include "arduino.h"
#include "config.h"
#include "safety.h"

bool error = false;
//bool connectionEstablished = false;

void startSafety(){
  pinMode(errorPin, INPUT_PULLUP);
}

void errorCheck(){
  if (digitalRead(errorPin) == HIGH){
    error == true;
    //motorsEnabled = false;
  }
  else{
    error == false;
  }
}
