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

#include <Arduino.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <SD.h>

#define SOFT_SERIAL_IN          6
#define SOFT_SERIAL_OUT         7

SoftwareSerial tinySerial(SOFT_SERIAL_IN, SOFT_SERIAL_OUT);
File logFile;
boolean logOk = false;

int readline(int readch, char *buffer, int len) {
  static int pos = 0;
  int rpos;

  if (readch > 0) {
    switch (readch) {
      case '\n': // Ignore new-lines
        break;
      case '\r': // Return on CR
        rpos = pos;
        pos = 0; // Reset position index ready for next time
        return rpos;
      default:
        if (pos < len - 1) {
          buffer[pos++] = readch;
          buffer[pos] = 0;
        }
        break;
    }
  }
  // No end of line has been found, so return -1.
  return -1;
}

void setup() {
  Serial.begin(115200); // Init serial band rate
  tinySerial.begin(9600);

  Serial.print("Initializing SD card...");
  pinMode(SD_CHIP_SELECT_PIN, OUTPUT);

  if (!SD.begin(SD_CHIP_SELECT_PIN)) {
    Serial.println("Initialization failed!");
    return;
  }
  Serial.println("Initialization done.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  logFile = SD.open("lightlog.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (!logFile) {
    // if the file didn't open, print an error:
    Serial.println("Error opening file!");
  }

  logOk = true;
}

void loop() {
  static char buffer[80];

  if (readline(tinySerial.read(), buffer, 80) > 0) {
    Serial.println(buffer);

    if (logOk) {
      logFile.println(buffer);
      logFile.flush();
    }
  }
}

