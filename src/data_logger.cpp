#include "data_logger.h"

#include <SD.h>

#include "config.h"
#include "sensors.h"

// Global file object (actual definition)
File dataFile;

/**
 * @brief Initialize SD card and create data file with headers
 * @return true if successful, false otherwise
 */
bool initializeDataLogging() {
  pinMode(Pins::SD_CHIP_SELECT, OUTPUT);

  if (!SD.begin(Pins::SD_CHIP_SELECT)) {
    if (DEBUG_ENABLED) {
      Serial.println(F("⚠ ERROR: SD card initialization failed!"));
    }
    return false;
  }

  // Create/open data file and write headers
  dataFile = SD.open(Config::DATA_FILENAME, FILE_WRITE);
  if (dataFile) {
    dataFile.println();
    dataFile.println(F("=== ROCKET FLIGHT DATA LOG ==="));
    dataFile.print(F("Timestamp(ms),Pressure(Pa),Temperature(C),Altitude(m),"));
    dataFile.print(F("AccelX(m/s²),AccelY(m/s²),AccelZ(m/s²),AccelMag(m/s²),"));
    dataFile.println(F("GyroX(°/s),GyroY(°/s),GyroZ(°/s)"));
    dataFile.close();

    if (DEBUG_ENABLED) {
      Serial.println(F("✓ Data logging initialized"));
    }
    return true;
  }

  return false;
}

/**
 * @brief Write sensor data to SD card with error tracking
 */
void logDataToSD() {
  if (!systemState.sdCardReady) return;

  dataFile = SD.open(Config::DATA_FILENAME, FILE_WRITE);

  if (dataFile) {
    // Write data in CSV format
    dataFile.print(sensors.timestamp);
    dataFile.print(",");
    dataFile.print(sensors.pressure, 2);
    dataFile.print(",");
    dataFile.print(sensors.temperature, 2);
    dataFile.print(",");
    dataFile.print(sensors.altitude, 2);
    dataFile.print(",");
    dataFile.print(sensors.accelX, 3);
    dataFile.print(",");
    dataFile.print(sensors.accelY, 3);
    dataFile.print(",");
    dataFile.print(sensors.accelZ, 3);
    dataFile.print(",");
    dataFile.print(sensors.accelMagnitude, 3);
    dataFile.print(",");
    dataFile.print(sensors.gyroX, 3);
    dataFile.print(",");
    dataFile.print(sensors.gyroY, 3);
    dataFile.print(",");
    dataFile.println(sensors.gyroZ, 3);
    dataFile.close();

    // Reset failure counter on successful write
    systemState.sdWriteFailureCount = 0;

  } else {
    // Increment failure counter
    systemState.sdWriteFailureCount++;

    if (DEBUG_ENABLED) {
      Serial.print(F("⚠ ERROR: Failed to write to SD card ("));
      Serial.print(systemState.sdWriteFailureCount);
      Serial.println(F(" consecutive failures)"));
    }

    // Disable SD logging after threshold exceeded
    if (systemState.sdWriteFailureCount >= Config::MAX_SD_FAILURES) {
      systemState.sdCardReady = false;

      if (DEBUG_ENABLED) {
        Serial.println(
            F("⚠ CRITICAL: SD card disabled due to repeated failures"));
      }
    }
  }
}

/**
 * @brief Print sensor data to serial monitor for debugging
 */
void printDebugData() {
  if (!DEBUG_ENABLED) return;

  Serial.print(F("Time: "));
  Serial.print(sensors.timestamp);
  Serial.print(F(" | P: "));
  Serial.print(sensors.pressure, 1);
  Serial.print(F(" | T: "));
  Serial.print(sensors.temperature, 1);
  Serial.print(F(" | Alt: "));
  Serial.print(sensors.altitude, 1);
  Serial.print(F(" | Acc: ["));
  Serial.print(sensors.accelX, 2);
  Serial.print(F(", "));
  Serial.print(sensors.accelY, 2);
  Serial.print(F(", "));
  Serial.print(sensors.accelZ, 2);
  Serial.print(F("] |Mag: "));
  Serial.print(sensors.accelMagnitude, 2);
  Serial.print(F(" | Gyro: ["));
  Serial.print(sensors.gyroX, 1);
  Serial.print(F(", "));
  Serial.print(sensors.gyroY, 1);
  Serial.print(F(", "));
  Serial.print(sensors.gyroZ, 1);
  Serial.println(F("]"));
}