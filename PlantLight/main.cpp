/*
 *  This file is part of PlantLight application.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

/*
 * Built for Attiny85 1Mhz, using AVR USBasp programmer.
 * VERSION 0.2
 */

#include <Arduino.h>
#include <TinyWireM.h>

#include <BH1750FVI.h>
#include <DS1307RTC.h>

#define RELAY_SW                4       // Relay out pin
#define CMD_MAN                 1       // Manual light switch

USI_TWI bus; // TinyWireM instance (I2C bus)
BH1750FVI BH1750(bus);
RTC_DS1307 RTC(bus);

void setup() {
  Serial.begin(9600); // Init serial band rate

  pinMode(RELAY_SW, OUTPUT);
  pinMode(CMD_MAN, INPUT_PULLUP);

  // I2C begin() is called BEFORE sensors library "begin" methods:
  // it is called just once for all the sensors.
  bus.begin();

  // Sensors initialization
  // Light sensor
  BH1750.powerOn();

  // Real time clock
  RTC.sqw(0);
  if (!RTC.isrunning()) {
    Serial.println("RTC is NOT running!");
    RTC.adjust(DateTime(__DATE__, __TIME__));
  }

}

void log(DateTime dt, uint16_t lt) {
  Serial.print(dt.year(), DEC);
  Serial.print('/');
  Serial.print(dt.month(), DEC);
  Serial.print('/');
  Serial.print(dt.day(), DEC);
  Serial.print(' ');
  Serial.print(dt.hour(), DEC);
  Serial.print(':');
  Serial.print(dt.minute(), DEC);
  Serial.print(':');
  Serial.print(dt.second(), DEC);
  Serial.print(' ');
  Serial.println(lt);
}

boolean relayState = 0;
uint16_t lux = 0;
DateTime now;

void loop() {
  lux = BH1750.getLightIntensity(); // Get lux value
  now = RTC.now();

  log(now, lux);

  if (lux <= 10 || !digitalRead(CMD_MAN)) {
    relayState = true;
  } else relayState = false;

  digitalWrite(RELAY_SW, relayState);

  delay(10000);

}
