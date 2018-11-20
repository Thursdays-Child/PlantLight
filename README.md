# PLANT LIGHT 
### Automatic light switch based on light sensor and date/time
**version 2.0**
Upgraded PCB and logic with ATMega328p and Maxim DS3231M RTC. New schematich and new PCB. The temperature compensated clock
now wakes up the micro once per day, with time of day alarm. Added HM10 BLE connectivity module to send and receive date, time,
settings.

**version 1.2**
Added function to set time from an external input, with all the istructions needed: set draft info on RTC clock.
Linked against new ATTiny CORE library, that has changed the RX and TX pin for serial debug.

**version 1.1**

Added TimeZone library to account for DST (summer time). The TimeZone has been added to the DS1307RTC library,
and slightly modified to remove ARDUINO define, not present in my development environment. 
The timezone rules are saved into an EEPROM image (to save program memory on the small ATTiny85): the image is in 
Intel HEX format.

Library reference at https://github.com/JChristensen/Timezone

**version 1.0**

The project is an automatic switch for 2 LED lamps used to illuminate 
some plants in my house. On board there is a real time clock, to keep track of 
the actual date and time even on power loss, and the light sensor is placed 
on a separate breakout board. A relay switches the main power (230 V AC) to 
the LEDs. The micro is an Atmel ATTiny 85V.

The project is compiled for Atmel AVR, with standard AVR GCC toolchain. 
Eclipse was used to create and compile the projects, with the AVR Plugin.

A makefile is available to compile without Eclispe. 

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
