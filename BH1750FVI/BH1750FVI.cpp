/*
 * This library is for Digital Light sensor BH1750FVI,
 * uses I2C Communication protocol to interface with the sensor.
 *
 * Revision merged with library from Christopher Laws, March, 2013.
 * Credits for the first version to: Mohannad Rawashdeh, www.genotronex.com.
 *
 * Modified to use a private: USI_TWI &busI2C reference at the I2C bus
 * from the application.
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

#include "BH1750FVI.h"

BH1750FVI::BH1750FVI(USI_TWI &bus): busI2C(bus) {
  address = BH1750_I2CADDR_L;
  currentMode = BH1750_CONTINUOUS_HIGH_RES_MODE;
}

void BH1750FVI::powerOn(void) {
  writeToBus(BH1750_POWER_ON);      // Turn it On
  setMode(currentMode);
}

void BH1750FVI::sleep(void) {
  writeToBus(BH1750_POWER_DOWN);    // Turn it off, reset command won't work in this mode
}

void BH1750FVI::reset(void) {
  writeToBus(BH1750_POWER_ON);      // Turn it on again
  writeToBus(BH1750_RESET);         // Reset
}

void BH1750FVI::wakeUp() {
  // Power on command can be omitted
  setMode(currentMode);
}

void BH1750FVI::wakeUp(uint8_t mode) {
  setMode(mode);
}

void BH1750FVI::setAddress(uint8_t addPin, boolean add) {
  pinMode(addPin, OUTPUT);

  if (add) {
    address = BH1750_I2CADDR_H;
    digitalWrite(addPin, HIGH);
  } else {
    address = BH1750_I2CADDR_L;
    digitalWrite(addPin, LOW);
  }
}

void BH1750FVI::setMode(uint8_t mode) {
  switch (mode) {
    case BH1750_CONTINUOUS_HIGH_RES_MODE:
    case BH1750_CONTINUOUS_HIGH_RES_MODE_2:
    case BH1750_CONTINUOUS_LOW_RES_MODE:
    case BH1750_ONE_TIME_HIGH_RES_MODE:
    case BH1750_ONE_TIME_HIGH_RES_MODE_2:
    case BH1750_ONE_TIME_LOW_RES_MODE:
      // Apply a valid mode change
      writeToBus(mode);
      currentMode = mode;

      break;
    default:
      // Invalid measurement mode
#if BH1750_DEBUG == 1
      Serial.print("Invalid measurement mode: 0x");
      Serial.println(mode, HEX);
#endif
      break;
  }
}

uint16_t BH1750FVI::getLightIntensity(void) {
  uint16_t intensityValue;

  // With TinyWireM library beginTransmission() and endTransmission()
  // don't have be called!
  busI2C.requestFrom(address, 2);

  intensityValue = busI2C.receive();
  intensityValue <<= 8;
  intensityValue |= busI2C.receive();

  intensityValue = intensityValue / LUX_SCALE_COEF;

  return intensityValue;
}

void BH1750FVI::writeToBus(uint8_t DataToSend) {
  busI2C.beginTransmission(address);
  busI2C.send(DataToSend);
  busI2C.endTransmission();
}
