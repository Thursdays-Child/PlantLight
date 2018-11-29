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
 * VERSION 2.0b003
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

#define SERIAL_BAUD             9600    // Serial baud per second

// Pins
#define RTC_INT_SQW             4       // INT/SQW pin from RTC
#define RELAY_SW_OUT            5       // Relay out pin

#define SAMPLE_COUNT            10      // Light sample count (average measurement)
#define LUX_TH                  10      // Lux threshold
#define LUX_TH_HIST             5       // Lux threshold (hysteresis compensation)

//#define RTC_CONTROL 0x0E                // TODO remove
//#define RTC_STATUS 0x0F

// ################################# Constants ################################

// ################################# Variables ################################

TwoWire bus;                            // TinyWireM instance (I2C bus)
//BH1750FVI BH1750(bus);                  // Light sensor instance
DS3232RTC RTC(bus);                     // RTC clock instance

volatile bool rtcWakeUp = false;

static float luxSamples[SAMPLE_COUNT] = {};
static uint8_t readCounter = 0;
static boolean relayState = false, relayStateMem = false;
static struct tm utcTime, localTime;

//CET Time Zone (Rome, Berlin) -> UTC/GMT + 1
const TimeChangeRule CEST = {"CEST", Last, Sun, Mar, 2, 120};   // Central European Summer Time
const TimeChangeRule CET  = {"CET ", Last, Sun, Oct, 3, 60};    // Central European Standard Time
Timezone TZ(CEST, CET);                                         // Constructor to build object from DST Rules
TimeChangeRule *tcr;                                            // Pointer to the time change rule, use to get TZ abbreviations

// ############################################################################
// Sensor reading functions
// ###########################################################################

/*
 * Sample from light sensor.
 *
 * Return: the current mobile average value.
 */
float sample() {
  float tmpLux = 0.0;

  // Shift left values in sample array
  for (int i = 0; i < SAMPLE_COUNT - 1; ++i) {
    luxSamples[i] = luxSamples[i + 1];
    tmpLux += luxSamples[i];
  }

  //luxSamples[SAMPLE_COUNT - 1] = BH1750.getLightIntensity();  // TODO Get lux value

  luxSamples[SAMPLE_COUNT - 1] = 9.0;

  tmpLux += luxSamples[SAMPLE_COUNT - 1];

  if (readCounter < SAMPLE_COUNT) {
    readCounter++;
  }

  return tmpLux / readCounter;
}

/*
 * Cleans the mobile average array.
 */
void cleanLuxArray() {
  readCounter = 0;
  for (int i = 0; i < SAMPLE_COUNT; ++i) {
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
  for (int i = SAMPLE_COUNT / 2 + 1 ; i < SAMPLE_COUNT - 1; ++i) {
    if (luxSamples[i] <= LUX_TH) {
      low++;
    } else {
      high++;
    }
  }

  if (readCounter >= SAMPLE_COUNT / 2 + 1) {

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
  return (now.tm_hour >= 16 && now.tm_hour <= 22 && now.tm_min <= 10);
}

#ifdef DEBUG
void printLuxArray() {
  for (int i = 0; i < SAMPLE_COUNT; ++i) {
    Serial.print(luxSamples[i]);
    Serial.print(" ");
  }
}
#endif


// ############################################################################
// AVR specific functions
// ############################################################################

/*
 *  PCINT Interrupt Service Routine
 */
ISR(PCINT2_vect) {
  rtcWakeUp = true;
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

  // RTC connection check
  if ((retcode = RTC.checkCon()) != 0) {
    Serial.print("RTC err: ");
    Serial.println(retcode);

    // Exit application code to infinite loop
    exit(retcode);
  } else {
    // Real time clock set: UTC time!
    //makeTime(&utcTime, 2018, 11, 28, 20, 9, 45);
    //RTC.write(&utcTime);
    RTC.read(&utcTime);
    TZ.toLocal(&utcTime, &localTime, &tcr);

#ifdef DEBUG
    printTime(&utcTime, "UTC");
    printTime(&localTime, tcr->abbrev);
#endif

    // Set alarm to the RTC clock in UTC format!!
    RTC.setAlarm(ALM1_MATCH_HOURS, (utcTime.tm_sec + 5) % 60, utcTime.tm_min, utcTime.tm_hour, utcTime.tm_wday);
    // Clear the alarm flag
    RTC.alarm(ALARM_1);
    // Set the alarm interrupt
    RTC.alarmInterrupt(ALARM_1, true);
  }

  // Light Sensor initialization
//  BH1750.powerOn();
//  BH1750.setMode(BH1750_CONTINUOUS_HIGH_RES_MODE_2);
//  BH1750.setMtreg(200);                 // Set measurement time register to high value
//  delay(100);
//  BH1750.sleep();                       // Send light sensor to sleep

  // Power settings
  powerReduction();

  // Interrupt configuration
  PCMSK2 |= _BV(PCINT20);               // Pin change mask: listen to portD bit 4 (D4) (RTC_INT_SQW)
  PCMSK2 |= _BV(PCINT16);               // Pin change mask: listen to portD bit 0 (D0) (Serial RX)
  PCICR  |= _BV(PCIE2);                 // Enable PCINT interrupt on portD
}

void loop() {
#ifdef DEBUG
  Serial.println("Sleeping...");
  Serial.println();
  delay(500);
#endif
  systemSleep(SLEEP_MODE_PWR_DOWN);     // Send the unit to sleep

  if (rtcWakeUp) {
    rtcWakeUp = false;
    RTC.alarm(ALARM_1);                 // Necessary to reset the alarm flag on RTC!
  }

  RTC.read(&utcTime);
  TZ.toLocal(&utcTime, &localTime, &tcr);

#ifdef DEBUG
  printTime(&utcTime, "UTC");
  printTime(&localTime, tcr->abbrev);
#endif

  if (checkEnable(localTime)) {
    RTC.setAlarm(ALM1_MATCH_SECONDS, (utcTime.tm_sec + 3) % 60, 0, 0, 0);

//    // One time mode: the sensor reads and goes into sleep mode autonomously
//    BH1750.wakeUp(BH1750_ONE_TIME_HIGH_RES_MODE_2);
//
//    // Wait for the sensor to be fully awake.
//    delay(500);

    float lux = sample();
    relayState = checkLightCond(lux);

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
//    TODO set next alarm
//    makeTime(&utcTime, 2018, 11, 28, 20, 9, 45);
//    RTC.write(&utcTime);
//    RTC.setAlarm(ALM1_MATCH_HOURS, 50, 9, 20, utcTime.tm_wday);

    cleanLuxArray();
    relayState = false;
  }

  // Relay output update
  if (relayState != relayStateMem) {
    digitalWrite(RELAY_SW_OUT, relayState);
    relayStateMem = relayState;
  }
}
