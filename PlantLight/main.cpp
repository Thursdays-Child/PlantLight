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

#define BLU_STATE               2       // BLE module state pin
#define BLU_RESET               3       // BLE module reset pin

// Serial defines
#define SERIAL_BAUD             9600    // For AT mode and data mode (CC41, HM-10 and MLT-BT05)

#define RTC_INT_SQW             4       // INT/SQW pin from RTC
#define SER_RX                  0       // Pin PB0
#define SER_TX                  1       // Pin PB1 - tiny serial debug pin

#define WD_TICK_TIME            8       // Watchdog tick time [s] (check WD_MODE or datasheet)
#define WD_WAIT_EN_TIME         24      // Time [s] to count between two sensor read when enabled

#define SAMPLE_COUNT            10      // Light sample count (average measurement)
#define LUX_TH                  10      // Lux threshold
#define LUX_TH_HIST             5       // Lux threshold (hysteresis compensation)

// ####################### Constants ########################

// Watchdog interrupt sleep time
// 0=16ms, 1=32ms, 2=64ms, 3=125ms, 4=250ms, 5=500ms, 6=1 sec, 7=2 sec, 8=4 sec, 9=8sec
const float WD_MODE[10] = {0.016, 0.032, 0.064, 0.125, 0.25, 0.5, 1, 2, 4, 8};

// ####################### Variables ########################

TwoWire bus;                            // TinyWireM instance (I2C bus)
BH1750FVI BH1750(bus);                  // Light sensor instance
DS3232RTC RTC(bus);                     // RTC clock instance

volatile uint8_t wdCount = 0;

static float luxSamples[SAMPLE_COUNT] = {};
static uint8_t readCounter = 0;
static boolean relayState = false, relayStateMem = false;

//CET Time Zone (Rome, Berlin) -> UTC/GMT + 1
const TimeChangeRule CEST = {"CEST", Last, Sun, Mar, 2, 120};     // Central European Summer Time
const TimeChangeRule CET  = {"CET ", Last, Sun, Oct, 3, 60};      // Central European Standard Time
Timezone myTZ(CEST, CET);               // Constructor to build object from DST Rules
TimeChangeRule *tcr;                    // Pointer to the time change rule, use to get TZ abbreviations

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

  luxSamples[SAMPLE_COUNT - 1] = BH1750.getLightIntensity();  // Get lux value

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
  return (now.tm_hour >= 16 && now.tm_hour <= 22);
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
// Time manipulation functions
// ########################################################

#ifdef DEBUG
/*
 * Write to serial a digit with zero padding if needed.
 *
 * Param:
 * - int digits
 */
static void inline printDigits(int digits) {
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

/*
 * Writes to Serial the clock (date and time) in human readable format.
 *
 * Param:
 * - struct tm  *tm: struct tm from the time.h (avr-libc)
 * - const char *tz: string for the timezone, if not used set to NULL
 */
static void printTime(struct tm *tm, const char *tz = NULL) {
  printDigits(tm->tm_hour);
  Serial.print(':');
  printDigits(tm->tm_min);
  Serial.print(':');
  printDigits(tm->tm_sec);
  Serial.print(' ');
  Serial.print(tm->tm_mday);
  Serial.print('/');
  Serial.print(tm->tm_mon + 1);              // avr-libc time.h: months in [0, 11]
  Serial.print('/');
  Serial.print(tm->tm_year + 1900);          // avr-libc time.h: years since 1900

  Serial.print(' ');
  Serial.print(tm->tm_wday);

  if (tz != NULL) {
    Serial.print(' ');
    Serial.print(tz);
  }

  Serial.println();
}
#endif

/*
 * Converts the date/time to standard Unix epoch format, using time.h library (avr-libc)
 *
 * Param:
 * - int16_t YYYY: year (given as ex. 2017)
 * - int8_t MM: month [1, 12]
 * - int8_t DD: day of the month [1, 31]
 * - int8_t hh: hour [0, 23]
 * - int8_t mm: minute [0, 59]
 * - int8_t ss: second [0, 59]
 */
static time_t tmConvert_t(int16_t YYYY, int8_t MM, int8_t DD, int8_t hh, int8_t mm, int8_t ss) {
  struct tm tm;

  memset((void*) &tm, 0, sizeof(tm));

  tm.tm_year = YYYY - 1900;         // avr-libc time.h: years since 1900
  tm.tm_mon  = MM - 1;              // avr-libc time.h: months in [0, 11]
  tm.tm_mday = DD;
  tm.tm_hour = hh;
  tm.tm_min  = mm;
  tm.tm_sec  = ss;
  return mk_gmtime(&tm);
}

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
 * Setup watchdog timeout and start for AVR Mega CPUs. TODO
 */
void setupWatchdog(float tick) {
  // 0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms, 5=500ms, 6=1 sec, 7=2 sec, 8=4 sec, 9=8sec
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

/*
 * Stop watchdog count.
 */
void stopWatchdog() {
  MCUSR &= ~(1 << WDRF);
  // Write logical one to WDCE and WDE
  // Keep old prescaler setting to prevent unintentional time-out
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  // Turn off WDT
  WDTCSR = 0x00;
}

/*
 * Put the micro to sleep
 */
static void systemSleep() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_mode();

  // sleeping ...
  sleep_disable(); // wake up fully
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

  // BLE Reset high
  digitalWrite(BLU_RESET, HIGH);

  // RTC connection check
  if ((retcode = RTC.checkCon()) != 0) {
    Serial.print("RTC err: ");
    Serial.println(retcode);

    // Exit application code to infinite loop
    exit(retcode);
  }

  // Real time clock set: UTC time!
  time_t t;
  t = tmConvert_t(2018, 11, 18, 20, 00, 00);

  if (RTC.set(t) == 0) {
    // TODO set new alarm
    Serial.println("RTC OK");
  }

  // Sensors initialization
  // Light sensor
  BH1750.powerOn();
  BH1750.setMode(BH1750_CONTINUOUS_HIGH_RES_MODE_2);
  BH1750.setMtreg(200);                 // Set measurement time register to high value
  delay(100);
  BH1750.sleep();                       // Send light sensor to sleep

  // Setup watchdog timeout to 8 s (mode 9)
  setupWatchdog(WD_TICK_TIME);

  // Power settings
  powerReduction();

  // Interrupt configuration
  PCMSK2 |= _BV(PCINT20);               // Pin change mask: listen to portD bit 4 (D4) (RTC_INT_SQW)
  PCMSK2 |= _BV(PCINT16);               // Pin change mask: listen to portD bit 0 (D0) (Serial RX)
  PCICR  |= _BV(PCIE2);                 // Enable PCINT interrupt on portD
}

void loop() {
#ifdef DEBUG
  Serial.println();
  Serial.println("Sleeping...");
  delay(500);
#endif
  systemSleep();                        // Send the unit to sleep

  time_t utc = RTC.get();
  time_t local = myTZ.toLocal(utc, &tcr);

  struct tm tm_local;
  gmtime_r(&local, &tm_local);

#ifdef DEBUG
  struct tm tm_utc;
  gmtime_r(&utc, &tm_utc);

  delay(500);
  printTime(&tm_utc, "UTC");
  printTime(&tm_local, tcr->abbrev);
  Serial.print("wdCount = ");
  Serial.println(wdCount);
#endif

  if (checkEnable(tm_local)) {
    // Enable watchdog
    setupWatchdog(WD_TICK_TIME);

    // One time mode: the sensor reads and goes into sleep mode autonomously
    BH1750.wakeUp(BH1750_ONE_TIME_HIGH_RES_MODE_2);

    // Wait for the sensor to be fully awake.
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
    // Disable watchdog
    stopWatchdog();
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
