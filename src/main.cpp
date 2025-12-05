#include <Arduino.h>
#include <SD.h>
#include <DFRobot_WT61PC.h>
#include <SoftwareSerial.h>

/**
 * @file main.cpp
 * @brief Multi-sensor data logger for rocket flight telemetry with landing detection
 * @author Adil Wahab Bhatti
 * @version 4.0
 * @date 2025-12-05
 *
 * @description
 * This Arduino sketch implements a comprehensive data logging system for rocket flights,
 * capturing acceleration, gyroscope, temperature, and pressure data. Features automatic
 * landing detection with audible alert and real-time data storage to SD card.
 *
 * @hardware
 * - Arduino Uno/compatible microcontroller
 * - DFRobot WT61PC 6-axis IMU sensor
 * - LM35DT temperature sensor
 * - MPX4115A pressure sensor
 * - SD card module
 * - Buzzer for landing alert
 *
 * @dependencies
 * - SD Library (built-in)
 * - SoftwareSerial Library (built-in)
 * - DFRobot_WT61PC Library (https://github.com/DFRobotdl/DFRobot_WT61PC)
 *
 * @credits
 * Accelerometer integration code modified from DFRobot example:
 * - Original author: huyujie (yujie.hu@dfrobot.com)
 * - Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * - License: MIT License
 * - Source: https://github.com/DFRobot
 *
 * @license MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */

// ============================================================================
// CONFIGURATION & CONSTANTS
// ============================================================================

// Debug Configuration
const bool DEBUG_ENABLED = false; // Set to true for serial debugging

// Pin Definitions
namespace Pins
{
  const int SD_CHIP_SELECT = 17; // SD card CS pin
  const int IMU_RX = 10;         // IMU RXD connection
  const int IMU_TX = 11;         // IMU TXD connection
  const int TEMPERATURE = A3;    // LM35DT temperature sensor
  const int PRESSURE = A2;       // MPX4115A pressure sensor
  const int BUZZER = 13;         // Landing detection buzzer
}

// System Configuration
namespace Config
{
  const unsigned long BUZZER_ARM_TIME = 15000;   // Time to arm buzzer (ms)
  const unsigned long LOOP_DELAY = 200;          // Main loop delay (ms)
  const char *DATA_FILENAME = "flight_data.txt"; // SD card data file

  // Landing Detection Thresholds
  const float LANDING_ACC_MIN = 9.0;  // Minimum acceleration for landing (m/s²)
  const float LANDING_ACC_MAX = 12.0; // Maximum acceleration for landing (m/s²)

  // IMU Configuration
  const int IMU_FREQUENCY = FREQUENCY_5HZ; // Data output frequency
}

// Sensor Calibration Constants
namespace Calibration
{
  const float TEMP_VOLTAGE_REF = 5.0;           // Reference voltage for temperature sensor
  const float TEMP_SCALE_FACTOR = 100.0;        // LM35DT scale factor (10mV/°C)
  const float PRESSURE_OFFSET = 0.095;          // MPX4115A offset
  const float PRESSURE_SCALE = 0.000009;        // MPX4115A scale factor
  const float ALTITUDE_EXPONENT = 1.0 / 5.257;  // Barometric formula exponent
  const float REFERENCE_TEMP_K = 15.0 + 273.15; // Reference temperature in Kelvin
}

// ============================================================================
// GLOBAL OBJECTS & VARIABLES
// ============================================================================

// Hardware Objects
File dataFile;
SoftwareSerial imuSerial(Pins::IMU_RX, Pins::IMU_TX);
DFRobot_WT61PC imuSensor(&imuSerial);

// Sensor Data Structure
struct SensorReadings
{
  unsigned long timestamp = 0;

  // IMU Data
  float accelX = 0.0;
  float accelY = 0.0;
  float accelZ = 0.0;
  float accelMagnitude = 0.0;
  float gyroX = 0.0;
  float gyroY = 0.0;
  float gyroZ = 0.0;

  // Environmental Data
  float temperature = 0.0;
  float pressure = 0.0;
  float altitude = 0.0;
};

// System State
struct SystemState
{
  float referencePressure = 0.0; // Ground level pressure for altitude calculation
  bool sdCardReady = false;      // SD card initialization status
  bool buzzerArmed = false;      // Landing detection buzzer status
};

SensorReadings sensors;
SystemState systemState;

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * @brief Read temperature from LM35DT sensor
 * @param pin Analog pin connected to temperature sensor
 * @return Temperature in Celsius
 */
float readTemperature(int pin)
{
  float rawReading = analogRead(pin);
  float voltage = (rawReading / 1024.0) * Calibration::TEMP_VOLTAGE_REF;
  return voltage * Calibration::TEMP_SCALE_FACTOR;
}

/**
 * @brief Read pressure from MPX4115A sensor
 * @param pin Analog pin connected to pressure sensor
 * @return Pressure in Pascals
 */
float readPressure(int pin)
{
  int rawReading = analogRead(pin);
  float normalizedReading = rawReading / 1024.0;
  return (normalizedReading + Calibration::PRESSURE_OFFSET) / Calibration::PRESSURE_SCALE;
}

/**
 * @brief Calculate altitude from pressure differential
 * @param currentPressure Current atmospheric pressure (Pa)
 * @param referencePressure Ground level pressure (Pa)
 * @return Altitude in meters
 */
float calculateAltitude(float currentPressure, float referencePressure)
{
  if (referencePressure == 0.0 || currentPressure == 0.0)
    return 0.0;

  float pressureRatio = referencePressure / currentPressure;
  float altitudeFactor = pow(pressureRatio, Calibration::ALTITUDE_EXPONENT) - 1.0;
  return altitudeFactor * Calibration::REFERENCE_TEMP_K / 0.0065; // Standard lapse rate
}

/**
 * @brief Update all sensor readings
 */
void updateSensorReadings()
{
  sensors.timestamp = millis();

  // Read IMU data if available
  if (imuSensor.available())
  {
    sensors.accelX = imuSensor.Acc.X;
    sensors.accelY = imuSensor.Acc.Y;
    sensors.accelZ = imuSensor.Acc.Z;
    sensors.accelMagnitude = sqrt(pow(sensors.accelX, 2) +
                                  pow(sensors.accelY, 2) +
                                  pow(sensors.accelZ, 2));

    sensors.gyroX = imuSensor.Gyro.X;
    sensors.gyroY = imuSensor.Gyro.Y;
    sensors.gyroZ = imuSensor.Gyro.Z;
  }

  // Read environmental sensors
  sensors.temperature = readTemperature(Pins::TEMPERATURE);
  sensors.pressure = readPressure(Pins::PRESSURE);
  sensors.altitude = calculateAltitude(sensors.pressure, systemState.referencePressure);
}

/**
 * @brief Check for landing condition and control buzzer
 */
void checkLandingCondition()
{
  systemState.buzzerArmed = (sensors.timestamp >= Config::BUZZER_ARM_TIME);

  bool landingDetected = systemState.buzzerArmed &&
                         (sensors.accelMagnitude >= Config::LANDING_ACC_MIN) &&
                         (sensors.accelMagnitude <= Config::LANDING_ACC_MAX);

  digitalWrite(Pins::BUZZER, landingDetected ? HIGH : LOW);
}

// ============================================================================
// DATA LOGGING FUNCTIONS
// ============================================================================

/**
 * @brief Initialize SD card and create data file with headers
 * @return true if successful, false otherwise
 */
bool initializeDataLogging()
{
  pinMode(Pins::SD_CHIP_SELECT, OUTPUT);

  if (!SD.begin(Pins::SD_CHIP_SELECT))
  {
    if (DEBUG_ENABLED)
    {
      Serial.println("ERROR: SD card initialization failed!");
    }
    return false;
  }

  // Create/open data file and write headers
  dataFile = SD.open(Config::DATA_FILENAME, FILE_WRITE);
  if (dataFile)
  {
    dataFile.println();
    dataFile.println("=== ROCKET FLIGHT DATA LOG ===");
    dataFile.print("Timestamp(ms),Pressure(Pa),Temperature(C),Altitude(m),");
    dataFile.print("AccelX(m/s²),AccelY(m/s²),AccelZ(m/s²),AccelMag(m/s²),");
    dataFile.println("GyroX(°/s),GyroY(°/s),GyroZ(°/s)");
    dataFile.close();

    if (DEBUG_ENABLED)
    {
      Serial.println("✓ Data logging initialized");
    }
    return true;
  }

  return false;
}

/**
 * @brief Write sensor data to SD card
 */
void logDataToSD()
{
  if (!systemState.sdCardReady)
    return;

  dataFile = SD.open(Config::DATA_FILENAME, FILE_WRITE);
  if (dataFile)
  {
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
  }
  else if (DEBUG_ENABLED)
  {
    Serial.println("ERROR: Failed to write to SD card");
  }
}

/**
 * @brief Print sensor data to serial monitor for debugging
 */
void printDebugData()
{
  if (!DEBUG_ENABLED)
    return;

  Serial.print("Time: ");
  Serial.print(sensors.timestamp);
  Serial.print(" | P: ");
  Serial.print(sensors.pressure, 1);
  Serial.print(" | T: ");
  Serial.print(sensors.temperature, 1);
  Serial.print(" | Alt: ");
  Serial.print(sensors.altitude, 1);
  Serial.print(" | Acc: [");
  Serial.print(sensors.accelX, 2);
  Serial.print(", ");
  Serial.print(sensors.accelY, 2);
  Serial.print(", ");
  Serial.print(sensors.accelZ, 2);
  Serial.print("] |Mag: ");
  Serial.print(sensors.accelMagnitude, 2);
  Serial.print(" | Gyro: [");
  Serial.print(sensors.gyroX, 1);
  Serial.print(", ");
  Serial.print(sensors.gyroY, 1);
  Serial.print(", ");
  Serial.print(sensors.gyroZ, 1);
  Serial.println("]");
}

// ============================================================================
// INITIALIZATION
// ============================================================================

void setup()
{
  // Initialize serial communication for debugging
  if (DEBUG_ENABLED)
  {
    Serial.begin(9600);
    while (!Serial)
    {
      delay(10);
    }
    Serial.println("=== Rocket Data Logger Initializing ===");
  }

  // Initialize IMU sensor
  imuSerial.begin(9600);
  imuSensor.modifyFrequency(Config::IMU_FREQUENCY);

  if (DEBUG_ENABLED)
  {
    Serial.println("✓ IMU sensor initialized");
    Serial.print("✓ Data frequency set to ");

    switch (Config::IMU_FREQUENCY)
    {
    case FREQUENCY_5HZ:
      Serial.println("5 Hz");
      break;
    case FREQUENCY_10HZ:
      Serial.println("10 Hz");
      break;
    case FREQUENCY_20HZ:
      Serial.println("20 Hz");
      break;
    default:
      Serial.println("Unknown");
      break;
    }
  }

  // Initialize buzzer pin
  pinMode(Pins::BUZZER, OUTPUT);
  digitalWrite(Pins::BUZZER, LOW);

  // Get reference pressure for altitude calculation
  delay(1000); // Allow sensors to stabilize
  systemState.referencePressure = readPressure(Pins::PRESSURE);

  if (DEBUG_ENABLED)
  {
    Serial.print("✓ Reference pressure: ");
    Serial.print(systemState.referencePressure, 1);
    Serial.println(" Pa");
  }

  // Initialize data logging
  systemState.sdCardReady = initializeDataLogging();

  if (DEBUG_ENABLED)
  {
    if (systemState.sdCardReady)
    {
      Serial.println("✓ Data logging ready");
    }
    else
    {
      Serial.println("⚠ Data logging failed - continuing without SD card");
    }

    Serial.println("=== System Ready - Starting Data Collection ===");
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

void loop()
{
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
