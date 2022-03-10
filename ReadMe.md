# Stupid Robot Module

An Example of building firmware for a multi-motor, serial controlled module for robotics applications.

module consists of a hoverboard wheel driven by a JY-01 500W brushless motor driver with hall sensor feedback. Drive Direction is controlled by a brushed DC motor and incremental encoder configured as a servo. Homing is achieved using a digital hall sensor and a magnet attached to one of the servo gears. The servo motor is driven by a L298D H-bridge. IO signals to the motors and sensors are provided by an Arduino Nano. Control signals are sent to the Arduino via Serial connection from a companion computer.

## Repository Structure

- ./Firmware/stupid-robot-module
    - main folder for arduino firmware. configuration for the functions already implemented can be done via config.h. 

- ./Hardware
    - STLs and other drawings of the prototype. Have a look, but you are encouraged to roll your own.

- ./Kicad
    - The beginnings of a custom controller PCB

- ./Processing
    - Rudimentary user interface that connects from the host computer and controls the module via serial communication

If you want more information, raise an issue on the github repository :)


