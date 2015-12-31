# PLANT LIGHT 
### Automatic light switch based on light sensor and date/time
**version 0.25**

The project is an automatic switch for 2 LED lamps used to illuminate 
some plants in my house. On board there is a real time clock, to keep track of 
the actual date and time even on power loss, and the light sensor is placed 
on a separate breakout board. A relay switches the main power (230 V AC) to 
the LEDs. The micro is an Atmel ATTiny 85V.

The project is compiled for Atmel AVR,  with standard AVR GCC toolchain. 
Eclipse was used to create and compile the projects, with the AVR Plugin.

TODO A makefile is available to compile without Eclispe. 

This project consists of a main C++ file and some libraries (C/C++):

* BH1750FVI: control the light sensor (BH1750) to read the lux value and 
  put to sleep/wake up the sensor;
* DS1307: real time clock library, to read the actual date and time.
* TinyWireM: Wire library modified to work on small ATTiny, were the I2C management 
  hardware is not present. The library uses the internal USI hardware to communicate 
  over serial connection, and implement I2C protocol (reference http://playground.arduino.cc/Code/USIi2c).
* TinySerialDebug: library to use serial communication only for debug purpose 
  on the ATTiny. The configuration allows only 3 fixed speeds (baud/s) and the 
  communication is transmission only from the micro (reference https://code.google.com/archive/p/arduino-tiny/).

The sensors library have been modified to work with the same instance of the 
I2C bus control class, and each functionality has been tested with the hardware. 
The documentation folder contains:

* the bill of material;
* datasheet of used components;
* electrical drawings;

Some pictures can be found at https://goo.gl/photos/P42G4u1Tg8iYygQf6 
