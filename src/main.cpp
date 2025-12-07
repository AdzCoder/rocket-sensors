/**
 * @file main.cpp
 * @brief Multi-sensor data logger for rocket flight telemetry with landing
 * detection
 * @author Adil Wahab Bhatti
 * @version 4.2
 * @date 2025-12-07
 *
 * @description
 * This Arduino sketch implements a comprehensive data logging system for rocket
 * flights, capturing acceleration, gyroscope, temperature, and pressure data.
 * Features automatic landing detection with audible alert and real-time data
 * storage to SD card.
 *
 * @hardware
 * - Arduino Micro/compatible microcontroller
 * - DFRobot WT61PC 6-axis IMU sensor
 * - LM35DT temperature sensor
 * - MPX4115A pressure sensor
 * - SD card module
 * - Buzzer for landing alert
 *
 * @dependencies
 * - SD Library (built-in)
 * - SoftwareSerial Library (built-in)
 * - DFRobot_WT61PC Library (https://github.com/DFRobot/DFRobot_WT61PC)
 *
 * @credits
 * Accelerometer integration code modified from DFRobot example:
 * - Original author: huyujie (yujie.hu@dfrobot.com)
 * - Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * - License: MIT License
 * - Source: https://github.com/DFRobot
 *
 * @license MIT License
 */

#include <Arduino.h>

#include "config.h"
#include "data_logger.h"
#include "sensors.h"

// ============================================================================
// INITIALIZATION
// ============================================================================

void setup() {
  // Initialize serial communication for debugging
  if (DEBUG_ENABLED) {
    Serial.begin(9600);
    while (!Serial) {
      delay(10);
    }
    Serial.println(F("=== Rocket Data Logger Initializing ==="));
  }

  // Initialize IMU sensor
  initializeIMU();

  // Initialize buzzer pin
  pinMode(Pins::BUZZER, OUTPUT);
  digitalWrite(Pins::BUZZER, LOW);

  // Get reference pressure for altitude calculation
  initializeReferencePressure();

  // Initialize data logging
  systemState.sdCardReady = initializeDataLogging();

  if (DEBUG_ENABLED) {
    if (systemState.sdCardReady) {
      Serial.println(F("✓ Data logging ready"));
    } else {
      Serial.println(F("⚠ Data logging failed - continuing without SD card"));
    }
    Serial.println(F("=== System Ready - Starting Data Collection ==="));
    Serial.println();
  }

  // Brief startup indication
  digitalWrite(Pins::BUZZER, HIGH);
  delay(100);
  digitalWrite(Pins::BUZZER, LOW);
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Update all sensor readings
  updateSensorReadings();

  // Check for landing condition and control buzzer
  checkLandingCondition();

  // Log data to SD card
  logDataToSD();

  // Debug output
  printDebugData();

  // Wait before next reading
  delay(Config::LOOP_DELAY);
}
