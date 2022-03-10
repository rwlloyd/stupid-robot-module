#include "arduino.h"

void setServoPWM(){
  //After initial testing, you can hear the PWM frequency. Lets adjust the timers to make it only audible to wildlife :(
  // https://www.lsu.edu/deafness/HearingRange.html
  // https://www.etechnophiles.com/change-frequency-pwm-pins-arduino-uno/
  //  Code for Available PWM frequency for D9 & D10:
  TCCR1B = TCCR1B & B11111000 | B00000001; // set timer 1 divisor to 1 for PWM frequency of 31372.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000010; // for PWM frequency of 3921.16 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000011; // for PWM frequency of 490.20 Hz (The DEFAULT)
  //TCCR1B = TCCR1B & B11111000 | B00000100; // for PWM frequency of 122.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000101; // for PWM frequency of 30.64 Hz
}

void setPCRvectors(){
    // Oh no, we've run out of External interrupts on the arduino nano.
    // We need an extra one for the hall sensor, so the steering servo can home.
    // We still have pin change interrupts though! Just have to learn how to use them first.
    // https://thewanderingengineer.com/2014/08/11/arduino-pin-change-interrupts/
  
    //Turn on pin change interrupts on port D
    //PCICR |= 0b00000001;    // turn on port b
    //PCICR |= 0b00000010;    // turn on port c
    ///PCICR |= 0b00000100;    // turn on port d
    //PCICR |= 0b00000111;    // turn on all ports
    // Choose Which pins to Interrupt
    //PCMSK0 |= 0b00000001;    // turn on pin PB0, which is PCINT0, physical pin 14
    //PCMSK1 |= 0b00010000;    // turn on pin PC4, which is PCINT12, physical pin 27
    ///PCMSK2 |= 0b0001000;    // turn on pin PCINT20. Physical pin D4 on nano.
    // See other functions for the interrupt handling
}
