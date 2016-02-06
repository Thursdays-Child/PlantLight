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
 * VERSION 0.4
 */

#include <Arduino.h>
#include <TinyWireM.h>
#include <Time.h>

#include <BH1750FVI.h>
#include <DS1307RTC.h>

#define RELAY_SW_OUT               4        // Relay out pin
#define CMD_MAN_IN                 1        // Manual light switch

USI_TWI bus;                                // TinyWireM instance (I2C bus)
BH1750FVI BH1750(bus);                      // Light sensor instance
RTC_DS1307 RTC(bus);                        // RTC clock instance

long unsigned int startTime = 0;
tmDriftInfo di;

time_t timeProvider() {
  return RTC.get();
}

void printDigits(int digits) {
  // Utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

void SerialDisplayDateTime(time_t timeToDisplay) {
  Serial.print(year(timeToDisplay));
  Serial.print("/");
  Serial.print(month(timeToDisplay));
  Serial.print("/");
  Serial.print(day(timeToDisplay));
  Serial.print(" ");
  Serial.print(hourFormat12(timeToDisplay));
  printDigits(minute(timeToDisplay));
  printDigits(second(timeToDisplay));
  if (isAM(timeToDisplay)) {
    Serial.print(" AM ");
  } else {
    Serial.print(" PM ");
  }
  Serial.println(weekday(timeToDisplay));
}

void setTime() {
  tmElements_t tme;
  time_t newTime;

  tme.Year = 46;
  tme.Month = 2;
  tme.Day = 06;
  tme.Hour = 00;
  tme.Minute = 00;
  tme.Second = 30;
  newTime = makeTime(tme);
  RTC.set(newTime); // set the RTC and the system time to the received value
  setTime(newTime);

  tmDriftInfo diUpdate = RTC.read_DriftInfo(); // update DriftInfo in RTC
  diUpdate.DriftStart = newTime;
  RTC.write_DriftInfo(diUpdate);

  Serial.print("RTC Set to: ");
  SerialDisplayDateTime(newTime);
}

void getDriftInfo() {
//  di = RTC.read_DriftInfo();
//  Serial.println("");
//  if (di.DriftStart == 0 || di.DriftDays == 0 || di.DriftSeconds == 0) {
//    Serial.println("DriftInfo not set yet!");
//  }
//  Serial.println("*** DriftInfo Read from RTC Memory ***");
//  Serial.print("DriftStart   : ");
//  SerialDisplayDateTime(di.DriftStart);
//  Serial.print("DriftDays    : ");
//  Serial.println(di.DriftDays);
//  Serial.print("DriftSeconds : ");
//  Serial.println(di.DriftSeconds);
//  Serial.print("Day(s) since drift start: ");
//  Serial.println(float(now() - di.DriftStart) / float(SECS_PER_DAY));
//  long tmp = now() - di.DriftStart;
//  tmp *= di.DriftSeconds;
//  Serial.print("Your RTC has Drifted(seconds): ");
//  Serial.println(float(tmp) / float(SECS_PER_DAY * di.DriftDays));
}

void setup() {
  Serial.begin(9600);

  pinMode(RELAY_SW_OUT, OUTPUT);
  pinMode(CMD_MAN_IN, INPUT_PULLUP);

  // I2C begin() is called BEFORE sensors library "begin" methods:
  // it is called just once for all the sensors.
  bus.begin();

  // Sensors initialization
  // Light sensor
  BH1750.powerOn();
  BH1750.setMode(BH1750_CONTINUOUS_HIGH_RES_MODE_2);
  BH1750.setMtreg(250);

  // Real time clock
  setSyncProvider(timeProvider); // the function to get the time from the RTC
//  if (timeStatus() != timeSet)
//    Serial.println("Unable to sync with the RTC");
//  else Serial.println("RTC has set the system time");

  di = RTC.read_DriftInfo();
  di.DriftDays = 1000; // valid value 0 to 65,535
  di.DriftSeconds = 18000; // fine tune this until your RTC sync with your reference time, valid value -32,768 to 32,767
  RTC.write_DriftInfo(di); // once you're happy with the dri

  RTC.sqw(0);
  if (!RTC.isrunning()) {
    setTime();
  }

  startTime = millis();
}

boolean relayState = 0;
float lux = 0;

uint8_t btnInCounter = 0;

void loop() {
  if (!digitalRead(CMD_MAN_IN)) {
    if (btnInCounter < 10) {
      btnInCounter++;
    }
  } else {
    btnInCounter = 0;
  }

  if (millis() - startTime >= 1000) {
    startTime = millis();

    lux = BH1750.getLightIntensity(); // Get lux value

//    getDriftInfo();
    di = RTC.read_DriftInfo();

    time_t timeNow = now();
    time_t timeNow3 = now3(di);

    Serial.print("RTC NOW TIME:       ");
    SerialDisplayDateTime(timeNow);

    Serial.print("RTC NOW3 TIME:      ");
    SerialDisplayDateTime(timeNow3);

    Serial.print("LUX: ");
    Serial.println(lux);

    Serial.println("");
    Serial.println("");

    if (btnInCounter >= 10) {
      relayState = true;
    } else if (lux <= 10) {
      relayState = true;
    } else {
      relayState = false;
    }

    digitalWrite(RELAY_SW_OUT, relayState);
  }
}
