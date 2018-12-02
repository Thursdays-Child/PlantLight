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
 * Built for ATMega328P 1Mhz, using AVR Pololu programmer.
 * VERSION 2.0b004
 */

#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include <Arduino.h>
#include <Timezone.h>
#include <BH1750FVI.h>
#include <DS3232RTC.h>

#include "PowerUtils.h"
#include "TimeUtils.h"

#define SERIAL_BAUD          9600       // Serial baud per second

// Pins
#define RTC_INT_SQW             2       // INT/SQW pin from RTC: external interrupt
#define RELAY_SW_OUT            5       // Relay output pin
#define SET_TIME_IN             4       // Set time mode input pin: pin change interrupt
// TODO time mode LED

#define ENABLE_H               16       // Enable hour (default)
#define DISABLE_H              23       // Disable hour (default)
#define ENA_DIS_MIN             0       // Enable and disable minute (default)

#define POLLING_INT             3       // Polling interval (default)       [s]
#define SAMPLE_COUNT           10       // Light sample count (average measurement)
#define MAX_SAMPLE_COUNT       50       // Max light sample count
#define LUX_TH                 10       // Lux threshold
#define LUX_TH_HIST             5       // Lux threshold (hysteresis compensation)

//#define RTC_CONTROL 0x0E                // TODO remove
//#define RTC_STATUS 0x0F

// ################################# Constants ################################

// ################################# Variables ################################

TwoWire bus;                            // TinyWireM instance (I2C bus)
//BH1750FVI BH1750(bus);                  // Light sensor instance
DS3232RTC RTC(bus);                     // RTC clock instance

static float luxSamples[MAX_SAMPLE_COUNT] = {};
static uint8_t readCounter = 0;
static boolean relayState = false;
volatile boolean setTimeMode = false;
static struct tm utcTime, localTime;

//CET Time Zone (Rome, Berlin) -> UTC/GMT + 1
const TimeChangeRule CEST = {"CEST", Last, Sun, Mar, 2, 120};   // Central European Summer Time
const TimeChangeRule CET  = {"CET ", Last, Sun, Oct, 3, 60};    // Central European Standard Time
Timezone TZ(CEST, CET);                                         // Constructor to build object from DST Rules
TimeChangeRule *tcr;                                            // Pointer to the time change rule, use to get TZ abbreviations

// Options data
typedef struct {
  int8_t hhE, mmE, hhD, mmD;        // Time of day to enable or disable light
  uint8_t poll;                     // Polling interval when enabled       [s]
  uint8_t sampleCount;              // Light sample count (average measurement)
  uint8_t luxTh;                    // Lux threshold
  uint8_t luxThHys;                 // Lux threshold (hysteresis compensation)
} options;                          // Options structure

options opt;                        // Global application options

// ############################################################################
// Sensor reading functions
// ############################################################################

/*
 * Sample from light sensor.
 *
 * Return: the current mobile average value.
 */
float sample() {
  float tmpLux = 0.0;

  // Shift left values in sample array
  for (int i = 0; i < opt.sampleCount - 1; ++i) {
    luxSamples[i] = luxSamples[i + 1];
    tmpLux += luxSamples[i];
  }

  //luxSamples[opt.sampleCount - 1] = BH1750.getLightIntensity();  // TODO Get lux value

  luxSamples[opt.sampleCount - 1] = 9.0;

  tmpLux += luxSamples[opt.sampleCount - 1];

  if (readCounter < opt.sampleCount) {
    readCounter++;
  }

  return tmpLux / readCounter;
}

/*
 * Cleans the mobile average array.
 */
void cleanLuxArray() {
  readCounter = 0;
  for (int i = 0; i < opt.sampleCount; ++i) {
    luxSamples[i] = 0.0;
  }
}

/*
 * Checks the light condition to switch the relay on or off.
 *
 * Return: true to turn on the relay.
 */
boolean checkLightCond(float lux) {
  int low = 0, high = 0;
  for (int i = opt.sampleCount / 2 + 1 ; i < opt.sampleCount - 1; ++i) {
    if (luxSamples[i] <= LUX_TH) {
      low++;
    } else {
      high++;
    }
  }

  if (readCounter >= opt.sampleCount / 2 + 1) {
    // Turn light on
    if (high <= 2 && lux <= LUX_TH) {
      return true;
    }

    // Turn light off
    if (relayState && lux <= LUX_TH + LUX_TH_HIST) {
      return true;
    } else {
      return false;
    }
  }
  return false;
}

boolean checkEnable(const struct tm &now) {
  int minE = opt.hhE * 60 + opt.mmE;
  int minD = opt.hhD * 60 + opt.mmD;
  int nowM = now.tm_hour * 60 + now.tm_min;

  if (minE == minD) {
    return true;
  } else if (minE < minD) {
    return (minE <= nowM && nowM < minD);
  } else {
    return (minE <= nowM || nowM < minD);
  }
}

#ifdef DEBUG
void printLuxArray() {
  for (int i = 0; i < opt.sampleCount; ++i) {
    Serial.print(luxSamples[i]);
    Serial.print(" ");
  }
}
#endif

/**
 * Set the date and time by entering the following on the serial input:
 * year,month,day,hour,minute,second,
 *
 * Where
 *  year can be two or four digits,
 *  month is 1-12,
 *  day is 1-31,
 *  hour is 0-23, and
 *  minute and second are 0-59.
 *
 * Entering the final comma delimiter (after "second") will avoid a TODO
 * one-second timeout and will allow the RTC to be set more accurately.
 *
 * No validity checking is done, invalid values or incomplete syntax
 * in the input will result in an incorrect RTC setting.
 */
// TODO exit from serial (quit?)
boolean setTimeSerial() {
  int16_t YYYY;                 // Year in 4 digit format
  int8_t MM, DD, hh, mm, ss;
  time_t t;

  // Check for input to set the RTC, minimum length is 12, i.e. yy,m,d,h,m,s
  if (Serial.available() >= 12) {
    int y = Serial.parseInt();
    if (y >= 100 && y < 1000) {
      Serial.println(F("Error: Year must be two digits or four digits!"));
    } else {
      if (y >= 1000) {
        YYYY = y;
      } else {
        // (y < 100)
        YYYY = y + 2000;
      }

      MM = Serial.parseInt();
      DD = Serial.parseInt();
      hh = Serial.parseInt();
      mm = Serial.parseInt();
      ss = Serial.parseInt();

      t = makeTime(YYYY, MM, DD, hh, mm, ss);

      // Use the time_t value to ensure correct weekday is set
      RTC.set(t);

      // Set UTC application time
      memset((void*) &utcTime, 0, sizeof(utcTime));
      gmtime_r(&t, &utcTime);

      // Convert UTC time in local time
      TZ.toLocal(&utcTime, &localTime, &tcr);

      Serial.println(F("RTC set to: "));
      printTime(&utcTime, "UTC");
      printTime(&localTime, tcr->abbrev);
      Serial.println();

      // Dump any extraneous input
      while (Serial.available() > 0)
        Serial.read();
    }
  return true;
  }
return false;
}

// ############################################################################
// AVR specific functions
// ############################################################################

/*
 *  PCINT Interrupt Service Routines
 */
ISR(PCINT2_vect) {
  setTimeMode = true;
}

/*
 *  INT0 Interrupt Service Routines
 *
 *  External interrupt on PD2
 */
ISR(INT0_vect) {
  // Don't do anything here but we must include this
  // block of code otherwise the interrupt calls an
  // uninitialized interrupt handler.
}

/*
 * Set various power reduction options
 */
static void powerReduction() {
  // Disable digital input buffer on ADC pins
  DIDR0 = (1 << ADC5D) | (1 << ADC4D) | (1 << ADC3D) | (1 << ADC2D) | (1 << ADC1D) | (1 << ADC0D);

  // Disable digital input buffer on Analog comparator pins
  DIDR1 |= (1 << AIN1D) | (1 << AIN0D);

  // Disable Analog Comparator interrupt
  ACSR &= ~(1 << ACIE);

  // Disable Analog Comparator
  ACSR |= (1 << ACD);

  // Disable unused peripherals to save power
  // Disable ADC (ADC must be disabled before shutdown)
  ADCSRA &= ~(1 << ADEN);

  // Enable power reduction register except:
  // - USART0: for serial communications
  // - TWI module: for I2C communications
  // - TIMER0: for millis()
  PRR = 0xFF & (~(1 << PRUSART0)) & (~(1 << PRTWI)) & (~(1 << PRTIM0));
}

void setup() {
  uint8_t retcode;

  Serial.begin(SERIAL_BAUD);

  // I2C begin() is called BEFORE sensors library "begin" methods:
  // it is called just once for all the sensors.
  bus.begin();

  // Setup pins modes
  pinMode(RELAY_SW_OUT, OUTPUT);
  pinMode(RTC_INT_SQW, INPUT_PULLUP);
  pinMode(SET_TIME_IN, INPUT_PULLUP);

  // RTC connection check
  if ((retcode = RTC.checkCon()) != 0) {
    Serial.print("RTC err: ");
    Serial.println(retcode);

    // Exit application code to infinite loop
    exit(retcode);
  }

  // Default options load
  opt.hhE         = 21;//ENABLE_H;
  opt.mmE         = 0;//ENA_DIS_MIN;
  opt.hhD         = 21;//DISABLE_H;
  opt.mmD         = 1;//ENA_DIS_MIN;
  opt.poll        = 20; // TODO POLLING_INT;
  opt.sampleCount = SAMPLE_COUNT;
  opt.luxTh       = LUX_TH;
  opt.luxThHys    = LUX_TH_HIST;

  if (opt.sampleCount > MAX_SAMPLE_COUNT) {
    opt.sampleCount = MAX_SAMPLE_COUNT;
  }

  // Real time clock set: UTC time!
  //makeTime(&utcTime, 2018, 11, 28, 12, 59, 55);
  //RTC.write(&utcTime);
  RTC.read(&utcTime);
  TZ.toLocal(&utcTime, &localTime, &tcr);

#ifdef DEBUG
  printTime(&utcTime, "UTC");
  printTime(&localTime, tcr->abbrev);
#endif

  // Set alarm to the RTC clock in UTC format!!
  RTC.setAlarm(ALM2_MATCH_HOURS, opt.mmE, opt.hhE, 0);
  // Clear the alarm flag
  RTC.alarm(ALARM_2);
  // Set the alarm interrupt
  RTC.alarmInterrupt(ALARM_2, true);

  // Light Sensor initialization
//  BH1750.powerOn();
//  BH1750.setMode(BH1750_CONTINUOUS_HIGH_RES_MODE_2);
//  BH1750.setMtreg(200);                 // Set measurement time register to high value
//  delay(100);
//  BH1750.sleep();                       // Send light sensor to sleep

  // Power settings
  powerReduction();

  // Interrupt configuration
  PCMSK2 |= _BV(PCINT20);               // Pin change mask: listen to portD bit 4 (D4) (SET_TIME_IN)
  PCICR  |= _BV(PCIE2);                 // Enable PCINT interrupt on portD

  EICRA |= _BV(ISC01);                  // Set INT0 to trigger on falling edge (RTC_INT_SQW)
  EIMSK |= _BV(INT0);                   // Turns on INT0
}

void loop() {

  RTC.read(&utcTime);
  TZ.toLocal(&utcTime, &localTime, &tcr);

#ifdef DEBUG
  printTime(&utcTime, "UTC");
  printTime(&localTime, tcr->abbrev);
#endif

  if (setTimeMode) {
    // ############################# SET TIME MODE ############################
    Serial.println("Wake");
    delay(500);
    if (setTimeSerial()) {
      // TODO set alarm
      setTimeMode = false;
    }
  } else {
    //  TODO if (checkEnable(localTime)) {
    if (checkEnable(utcTime)) {
      RTC.setAlarm(ALM1_MATCH_SECONDS, (utcTime.tm_sec + opt.poll) % 60, 0, 0, 0);
      // Clear the alarm flag
      RTC.alarm(ALARM_1);
      // Set the alarm interrupt
      RTC.alarmInterrupt(ALARM_1, true);

      //    // One time mode: the sensor reads and goes into sleep mode autonomously
      //    BH1750.wakeUp(BH1750_ONE_TIME_HIGH_RES_MODE_2);
      //
      //    // Wait for the sensor to be fully awake.
      //    delay(500);

      float lux = sample();
      relayState = checkLightCond(lux);

      digitalWrite(RELAY_SW_OUT, relayState);

#ifdef DEBUG
      printLuxArray();
      Serial.print("= ");
      Serial.print(lux);
      Serial.print(" ");
      if (relayState)
        Serial.println("ON");
      else Serial.println("OFF");
#endif
    } else {
      // Reset the alarm interrupt
      RTC.alarmInterrupt(ALARM_1, false);

      cleanLuxArray();
      digitalWrite(RELAY_SW_OUT, false);
      relayState = false;
    }

#ifdef DEBUG
    Serial.println("Sleeping...");
    Serial.println();
    delay(100);
#endif
    systemSleep(SLEEP_MODE_PWR_DOWN);     // Send the unit to sleep

    RTC.alarm(ALARM_1);                   // Necessary to reset the alarm flag on RTC!
    RTC.alarm(ALARM_2);                   // Necessary to reset the alarm flag on RTC!
  }
}
