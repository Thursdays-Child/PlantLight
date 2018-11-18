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

#define RELAY_SW_OUT            4       // Relay out pin
#define CMD_MAN_IN              1       // Manual light switch

#define SER_RX                  0       // Pin PB0
#define SER_TX                  1       // Pin PB1 - tiny serial debug pin

#define WD_TICK_TIME            8       // Watchdog tick time [s] (check WD_MODE or datasheet)
#define WD_WAIT_EN_TIME         24      // Time [s] to count between two sensor read when enabled

#define SAMPLE_COUNT            10      // Light sample count (average measurement)
#define LUX_TH                  10      // Lux threshold
#define LUX_TH_HIST             5       // Lux threshold (hysteresis compensation)

#define RULES_TZ_ADD            0       // Timezone rules EEPROM start address

// ####################### Prototypes #######################

// Setup watchdog wakeup interrupt
// tick: watchdog tick time [s] (check WD_MODE or datasheet)
void setupWatchdog(float tick);

// Set system into the sleep state,
// system wakes up when watchdog is timed out
void systemSleep();

// Provides a time_t to Time library, wrapper for call
// to RTC.get(), which is not a static function (class member)
time_t timeProvider();

// Checks the current time of day and month of the year.
// Returns true if the light is now enabled.
boolean checkEnable(const struct tm &now);

// Checks actual lux value. Returns true if the conditions
// to turn lights on are met.
boolean checkLightCond(float lux);

// Sample some lux value and calculate the running average over time
float sample();

// Cleans the array of sensor measurement
void cleanLuxArray();

// Set system clock in UTC format.
void setTimeOnInput(int hr, int min, int sec, int dy, int mnth, int yr);

#ifdef DEBUG
// Prints the array of sensor measurement - DEBUG only
void printLuxArray();

// Print an integer in "00" format (with leading zero).
// Input value assumed to be between 0 and 99.
void sPrintI00(int val);

// Print an integer in ":00" format (with leading zero).
// Input value assumed to be between 0 and 99.
void sPrintDigits(int val);

// Function to print time with time zone
void printTime(struct tm *tm, char *tz);
#endif

// ####################### Constants ########################

// Watchdog interrupt sleep time
// 0=16ms, 1=32ms, 2=64ms, 3=125ms, 4=250ms, 5=500ms, 6=1 sec, 7=2 sec, 8=4 sec, 9=8sec
const float WD_MODE[10] = {0.016, 0.032, 0.064, 0.125, 0.25, 0.5, 1, 2, 4, 8};

// ####################### Variables ########################

TwoWire bus;                            // TinyWireM instance (I2C bus)
BH1750FVI BH1750(bus);                  // Light sensor instance
DS3232RTC RTC(bus);                    // RTC clock instance

volatile uint8_t wdCount = 0;

// Watchdog interrupt count before reading light value
// wd count * wd interval == 4 * 8 s = 32 s
uint16_t wdMatch = 0;
static float luxSamples[SAMPLE_COUNT] = {};
static uint8_t readCounter = 0;
static boolean relayState = false, relayStateMem = false;

//CET Time Zone (Rome, Berlin) -> UTC/GMT + 1
//const TimeChangeRule CEST = {"CEST", Last, Sun, Mar, 2, 120};     // Central European Summer Time
//const TimeChangeRule CET  = {"CET ", Last, Sun, Oct, 3, 60};      // Central European Standard Time
Timezone myTZ(RULES_TZ_ADD);            // Rules stored at EEPROM address RULES_TZ_ADD
TimeChangeRule *tcr;                    // Pointer to the time change rule, use to get TZ abbreviations

// ####################### Functions ########################

void setupWatchdog(float tick) {
  // 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
  // 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
  uint8_t wdreg, wdmode = 0;

  for (uint8_t i = 0; i < 10; ++i) {
    if (WD_MODE[i] == tick) {
      wdmode = i;
    }
  }

  if (wdmode > 9)
    wdmode = 9;
  wdreg = wdmode & 7;

  if (wdmode > 7)
    wdreg |= (1 << 5);
  wdreg |= (1 << WDCE);

  // Clear the reset flag.
  MCUSR &= ~(1 << WDRF);

  // Start timed sequence: in order to change WDE or the prescaler, we need to
  // set WDCE (This will allow updates for 4 clock cycles).
  WDTCSR |= (1 << WDCE) | (1 << WDE);

  // Set new watchdog timeout prescaler value.
  WDTCSR = wdreg;

  // Enable the WD interrupt (note no reset).
  WDTCSR |= _BV(WDIE);
}

void systemSleep() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // Sleep mode is set here
  sleep_enable();
  sleep_mode();                         // System actually sleeps here
  sleep_disable();                      // System continues execution here when watchdog timed out
}

// Watchdog Interrupt Service Routine
ISR(WDT_vect) {
  wdCount++;
}

// PCINT Interrupt Service Routine (unused)
ISR(PCINT0_vect) {
  // Don't do anything here but we must include this
  // block of code otherwise the interrupt calls an
  // uninitialized interrupt handler.
}

time_t timeProvider() {
  return RTC.get();
}

float sample() {
  float tmpLux = 0.0;
  // Shift left values in sample array
  for (int i = 0; i < SAMPLE_COUNT - 1; ++i) {
    luxSamples[i] = luxSamples[i + 1];
    tmpLux += luxSamples[i];
  }

  luxSamples[SAMPLE_COUNT - 1] = BH1750.getLightIntensity();  // Get lux value

  tmpLux += luxSamples[SAMPLE_COUNT - 1];

  if (readCounter < SAMPLE_COUNT) {
    readCounter++;
  }

  return tmpLux / readCounter;
}

#ifdef DEBUG
void printLuxArray() {
  for (int i = 0; i < SAMPLE_COUNT; ++i) {
    Serial.print(luxSamples[i]);
    Serial.print(" ");
  }
}
#endif

void cleanLuxArray() {
  readCounter = 0;
  for (int i = 0; i < SAMPLE_COUNT; ++i) {
    luxSamples[i] = 0.0;
  }
}

boolean checkEnable(const struct tm &now) {
  return (now.tm_hour >= 16 && now.tm_hour <= 22);
}

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

#ifdef DEBUG
void sPrintI00(int val) {
    if (val < 10) Serial.print('0');
    Serial.print(val, DEC);
    return;
}

void sPrintDigits(int val) {
    Serial.print(':');
    if(val < 10) Serial.print('0');
    Serial.print(val, DEC);
}

void printTime(struct tm *tm, char *tz) {
    sPrintI00(tm->tm_hour);
    sPrintDigits(tm->tm_min);
    sPrintDigits(tm->tm_sec);
    Serial.print(' ');
    Serial.print(tm->tm_wday);
    Serial.print(' ');
    sPrintI00(tm->tm_mday);
    Serial.print(' ');
    Serial.print(tm->tm_mon);
    Serial.print(' ');
    Serial.print(tm->tm_year);
    Serial.print(' ');
    Serial.print(tz);
    Serial.println();
}
#endif

/**
 * Set the system clock in UTC format.
 * Used only once to set the time with the edge of an external input pin.
 */
void setTimeOnInput(int hr, int min, int sec, int mday, int mon, int year) {
  struct tm tm;
  tm.tm_hour = hr;
  tm.tm_min  = min;
  tm.tm_sec  = sec;
  // TODO wday
  tm.tm_mday = mday;
  tm.tm_mon  = mon;
  tm.tm_year = year;

  RTC.write(&tm);

#ifdef DEBUG
  printTime(&tm, "UTC");
#endif
}

void setup() {
#ifdef DEBUG
  Serial.begin(9600);
#endif

  pinMode(RELAY_SW_OUT, OUTPUT);
  pinMode(CMD_MAN_IN, INPUT_PULLUP);

  // Setup watchdog timeout to 8 s (mode 9)
  setupWatchdog(WD_TICK_TIME);

  // Watchdog interrupt count before reading light value
  // example: wd count * wd interval == 4 * 8 s = 32 s
  // Start with "enable" time tick and then check the current time
  wdMatch = (uint16_t) WD_WAIT_EN_TIME / WD_TICK_TIME;

  // TODO Setup Pin change interrupt on PCINT1
  //GIMSK |= _BV(PCIE);
  //PCMSK |= _BV(PCINT1);

  // Set the internal registers to reduce power consumes
  ACSR   = (1 << ACD);                  // Shut down the analog comparator
  MCUCR |= (1 << BODS);                 // BOD disabled

  // I2C begin() is called BEFORE sensors library "begin" methods:
  // it is called just once for all the sensors.
  bus.begin();

  // Sensors initialization
  // Light sensor
  BH1750.powerOn();
  BH1750.setMode(BH1750_CONTINUOUS_HIGH_RES_MODE_2);
  BH1750.setMtreg(200);                 // Set measurement time register to high value
  delay(100);
  BH1750.sleep();                       // Send light sensor to sleep

  // Real time clock TODO
  // TODO set new alarm
}

void loop() {
  systemSleep();                                    // Send the unit to sleep

  time_t utc = RTC.get();
  time_t local = myTZ.toLocal(utc, &tcr);

  struct tm tm_local;
  gmtime_r(&local, &tm_local);

#ifdef DEBUG
  struct tm tm_utc;
  gmtime_r(&utc, &tm_utc);
  Serial.println();
  printTime(&tm_utc, "UTC");
  printTime(&tm_local, tcr->abbrev);

  delay(1000);
#endif

  if (checkEnable(tm_local)) {
    // TODO enable watchdog

    // One time mode: the sensor reads and goes into sleep mode autonomously
    BH1750.wakeUp(BH1750_ONE_TIME_HIGH_RES_MODE_2);

    // Wait for the sensor to be fully awake.
    // Less than 500ms is not enough when the program doesn't write on the
    // serial output
    delay(500);

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
    // TODO disable watchdog
    // TODO set new alarm

    cleanLuxArray();
    relayState = false;
  }

  // Relay output update
  if (relayState != relayStateMem) {
    digitalWrite(RELAY_SW_OUT, relayState);
    relayStateMem = relayState;
  }
}
