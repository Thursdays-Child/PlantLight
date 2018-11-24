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
 * Built for ATMega 1Mhz, using AVR Pololu programmer.
 * VERSION 2.0b001
 */

#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include <Arduino.h>
#include <Timezone.h>                   // https://github.com/JChristensen/Timezone
#include <BH1750FVI.h>
#include <DS3232RTC.h>
#include <TwoWire.h>

#include "PowerUtils.h"
#include "TimeUtils.h"

#define RELAY_SW_OUT            5       // Relay out pin

#define BLU_STATE               2       // BLE module state pin
#define BLU_RESET               3       // BLE module reset pin

// Serial defines
#define SERIAL_BAUD             9600    // For AT mode and data mode (CC41, HM-10 and MLT-BT05)

#define RTC_INT_SQW             4       // INT/SQW pin from RTC
#define SER_RX                  0       // Pin PB0
#define SER_TX                  1       // Pin PB1 - tiny serial debug pin

#define WD_MODE                 6       // Watchdog mode (check WD_MODE or datasheet)
#define WD_TICK_TIME            1       // Watchdog tick time [s] (check WD_MODE or datasheet)
#define WD_SAMPLE_COUNT         3       // Watchdog tick to count between two sensor read when enabled

#define SAMPLE_COUNT            10      // Light sample count (average measurement)
#define LUX_TH                  10      // Lux threshold
#define LUX_TH_HIST             5       // Lux threshold (hysteresis compensation)

// ####################### Constants ########################

// ####################### Variables ########################

TwoWire bus;                            // TinyWireM instance (I2C bus)
//BH1750FVI BH1750(bus);                  // Light sensor instance
DS3232RTC RTC(bus);                     // RTC clock instance

volatile uint8_t wdCount = 0;

static float luxSamples[SAMPLE_COUNT] = {};
static uint8_t readCounter = 0;
static boolean relayState = false, relayStateMem = false, wake = false;
struct tm systemTime;

//CET Time Zone (Rome, Berlin) -> UTC/GMT + 1
const TimeChangeRule CEST = {"CEST", Last, Sun, Mar, 2, 120};   // Central European Summer Time
const TimeChangeRule CET  = {"CET ", Last, Sun, Oct, 3, 60};    // Central European Standard Time
Timezone TZ(CEST, CET);                                         // Constructor to build object from DST Rules
TimeChangeRule *tcr;                                            // Pointer to the time change rule, use to get TZ abbreviations

// ########################################################
// Sensor reading functions
// ########################################################

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
  return (now.tm_hour >= 16 && now.tm_hour <= 22 && now.tm_min <= 9);
}

#ifdef DEBUG
void printLuxArray() {
  for (int i = 0; i < SAMPLE_COUNT; ++i) {
    Serial.print(luxSamples[i]);
    Serial.print(" ");
  }
}
#endif


// ########################################################
// AVR specific functions
// ########################################################

/*
 * Watchdog Interrupt Service Routine
 */
ISR(WDT_vect) {
  wdCount++;
}

/*
 *  PCINT Interrupt Service Routine (unused)
 */
ISR(PCINT2_vect) {
  wdCount = 0;
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

  // TODO  1Mhz clock?

  Serial.begin(SERIAL_BAUD);

  // I2C begin() is called BEFORE sensors library "begin" methods:
  // it is called just once for all the sensors.
  bus.begin();

  // Setup pins modes
  pinMode(RELAY_SW_OUT, OUTPUT);
  pinMode(RTC_INT_SQW, INPUT);
  pinMode(BLU_RESET, OUTPUT);

  // BLE Reset high
  digitalWrite(BLU_RESET, HIGH);

  // RTC connection check
  if ((retcode = RTC.checkCon()) != 0) {
    Serial.print("RTC err: ");
    Serial.println(retcode);

    // Exit application code to infinite loop
    exit(retcode);
  } else {
    //  // Real time clock set: UTC time!
    time_t tSet = makeTime(2018, 11, 22, 21, 9, 00);
    RTC.set(tSet);
    TZ.toLocal(tSet, &systemTime, &tcr);

#ifdef DEBUG
  printTime(tSet, "UTC");
  printTime(&systemTime, tcr->abbrev);
#endif

    // Set alarm to the RTC clock in UTC format!!
    RTC.setAlarm(ALM1_EVERY_SECOND, systemTime.tm_sec, systemTime.tm_min, systemTime.tm_hour, systemTime.tm_wday);
    RTC.alarmInterrupt(ALARM_1, true);
  }

  // Sensors initialization
  // Light sensor
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

  // Necessary to reset the alarm flag on RTC!
  if (RTC.alarm(ALARM_1) || RTC.alarm(ALARM_2)) {
#ifdef DEBUG
    Serial.println("Wake up from alarm");
#endif
  }

  time_t utc = RTC.get();
  TZ.toLocal(utc, &systemTime, &tcr);

#ifdef DEBUG
  printTime(utc, "UTC");
  printTime(&systemTime, tcr->abbrev);
#endif


  if (checkEnable(systemTime)) {
    // Setup watchdog timeout to 8 s (mode 9)
    setupWatchdog(WD_MODE);

    // Reset alarm interrupt from RTC
    RTC.alarmInterrupt(ALARM_1, false);

    if (wdCount >= WD_SAMPLE_COUNT) {
      wdCount = 0;

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
    }

//#ifdef DEBUG
//      Serial.print("wdCount: ");
//      Serial.println(wdCount);
//#endif
    wake = true;
  } else {
    // Disable watchdog
    stopWatchdog();

    if (wake) {
      RTC.setAlarm(ALM2_EVERY_MINUTE, systemTime.tm_sec, systemTime.tm_min, systemTime.tm_hour, systemTime.tm_wday);
      RTC.alarmInterrupt(ALARM_2, true);
      wake = false;
    }

    cleanLuxArray();
    relayState = false;
  }

  // Relay output update
  if (relayState != relayStateMem) {
    digitalWrite(RELAY_SW_OUT, relayState);
    relayStateMem = relayState;
  }
}
