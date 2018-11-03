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
 * VERSION 1.2
 */

#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/iotnx5.h>
#include <avr/pgmspace.h>

#include <Arduino.h>
#include <TinyWireM.h>
#include <Timezone.h>                   // https://github.com/JChristensen/Timezone
#include <BH1750FVI.h>
#include <DS1307RTC.h>

#define RELAY_SW_OUT            4       // Relay out pin
#define CMD_MAN_IN              1       // Manual light switch

#define SER_RX                  0       // Pin PB0
#define SER_TX                  1       // Pin PB1 - tiny serial debug pin

#define WD_TICK_TIME            8       // Watchdog tick time [s] (check WD_MODE or datasheet)
#define WD_WAIT_EN_TIME         24      // Time [s] to count between two sensor read when enabled
#define WD_WAIT_DIS_TIME        200     // Time [s] to check if the current time enables the system

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
boolean checkEnable(const time_t &now);

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
void printTime(time_t t, char *tz);
#endif


// ####################### Constants ########################

// Watchdog interrupt sleep time
// 0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms, 5=500ms, 6=1 sec, 7=2 sec, 8=4 sec, 9=8sec
const float WD_MODE[10] = {0.016, 0.032, 0.064, 0.128, 0.25, 0.5, 1, 2, 4, 8};


// ####################### Variables ########################

USI_TWI bus;                            // TinyWireM instance (I2C bus)
BH1750FVI BH1750(bus);                  // Light sensor instance
RTC_DS1307 RTC(bus);                    // RTC clock instance

tmDriftInfo di;

volatile uint8_t wdCount = 0;

// Watchdog interrupt count before reading light value
// wd count * wd interval == 4 * 8 s = 32 s
uint16_t wdMatch = 0;
boolean first = false;
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

  MCUSR &= ~(1 << WDRF);

  // Start timed sequence
  WDTCR |= (1 << WDCE) | (1 << WDE);

  // Set new watchdog timeout value
  WDTCR = wdreg;
  WDTCR |= _BV(WDIE);
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

boolean checkEnable(const time_t &now) {
  return (hour(now) >= 16 && hour(now) <= 22);
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

/**
 * Set the system clock in UTC format.
 * Used only once to set the time with the edge of an external input pin.
 */
void setTimeOnInput(int hr, int min, int sec, int dy, int mnth, int yr) {
  setTime(hr, min, sec, dy, mnth, yr);

#ifdef DEBUG
  Serial.println(getSystemTime());
#endif

  time_t systemTime = getSystemTime();
  RTC.set(systemTime);
  tmDriftInfo diUpdate = RTC.read_DriftInfo(); // Update DriftInfo in RTC
  diUpdate.DriftStart = systemTime;

  RTC.write_DriftInfo(diUpdate);
  setDriftInfo(diUpdate);
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

void printTime(time_t t, char *tz) {
    sPrintI00(hour(t));
    sPrintDigits(minute(t));
    sPrintDigits(second(t));
    Serial.print(' ');
    Serial.print(weekday(t));
    Serial.print(' ');
    sPrintI00(day(t));
    Serial.print(' ');
    Serial.print(month(t));
    Serial.print(' ');
    Serial.print(year(t));
    Serial.print(' ');
    Serial.print(tz);
    Serial.println();
}
#endif

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

  // Setup Pin change interrupt on PCINT1
  GIMSK |= _BV(PCIE);
  PCMSK |= _BV(PCINT1);

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

  // Real time clock
  setSyncProvider(timeProvider);        // Pointer to function to get the time from the RTC
  setSyncInterval(WD_WAIT_DIS_TIME);    // Set system time at every sensor read

  // Update drift info from RTC
  di = RTC.read_DriftInfo();
  di.DriftDays = 1000;                  // RTC drift in seconds/day. 18 s per day is 18000/1000
  di.DriftSeconds = 27857;
  RTC.write_DriftInfo(di);
  setDriftInfo(di);
}

void loop() {

#ifdef DEBUG
  Serial.println();
  time_t utc = now();
  printTime(utc, "UTC");
  time_t local = myTZ.toLocal(utc, &tcr);
  printTime(local, tcr->abbrev);
  delay(1000);
#endif

  systemSleep();                                    // Send the unit to sleep

  // Manual command to turn light on
  if (!digitalRead(CMD_MAN_IN)) {
    if (!first) {
      // Used only once to set the time with external input
      setTimeOnInput(17, 46, 00, 3, 11, 2018);

      first = true;
      cleanLuxArray();
      relayState = relayStateMem = true;
      digitalWrite(RELAY_SW_OUT, relayState);
    }
  } else  {
    // When first == true the first cycle in automatic mode is executed,
    // after manual mode
    
    if (wdCount >= wdMatch || first) {
      first = false;
      wdCount = 0;

      // Conversion from UTC (system clock) to local time
      time_t utc = now();
      time_t local = myTZ.toLocal(utc, &tcr);

      if (checkEnable(local)) {
        wdMatch = (uint16_t) WD_WAIT_EN_TIME / WD_TICK_TIME;

        // One time mode: the sensor reads and goes into sleep mode autonomously
        BH1750.wakeUp(BH1750_ONE_TIME_HIGH_RES_MODE_2);

        // Wait for the sensor to be fully awake.
        // Less than 500ms is not enough when the program doesn't write on the 
        // serial output
        delay(500);

        float lux = sample();
        relayState = checkLightCond(lux);

#ifdef DEBUG
        printTime(local, tcr->abbrev);
        printLuxArray();
        Serial.print("= ");
        Serial.print(lux);
        Serial.print(" ");
        if (relayState)
          Serial.println("ON");
        else Serial.println("OFF");
#endif
      } else {
        // Set longer period to check the current time.
        // This requires less reads on the RTC when the light management is not
        // enabled. The accuracy of time when the system is enabled depends on
        // the watchdog tick (8 s is the longest) and on the wait time.
        // For this application the accuracy of one second is not needed:
        // - the "enable" event could be late up to WD_WAIT_DIS_TIME seconds.
        // - the "disable" event could be late up to WD_WAIT_EN_TIME seconds.
        wdMatch = (uint16_t) WD_WAIT_DIS_TIME / WD_TICK_TIME;

        cleanLuxArray();
        relayState = false;
      }

      // Relay output update
      if (relayState != relayStateMem) {
        digitalWrite(RELAY_SW_OUT, relayState);
        relayStateMem = relayState;
      }
    }
  }
}
