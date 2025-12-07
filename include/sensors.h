#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <DFRobot_WT61PC.h>
#include <SoftwareSerial.h>

// ============================================================================
// SENSOR DATA STRUCTURES
// ============================================================================

/**
 * @brief Structure to hold all sensor readings
 */
struct SensorReadings {
  unsigned long timestamp = 0;  // System uptime in milliseconds

  // IMU Data
  float accelX = 0.0;          // X-axis acceleration (m/s²)
  float accelY = 0.0;          // Y-axis acceleration (m/s²)
  float accelZ = 0.0;          // Z-axis acceleration (m/s²)
  float accelMagnitude = 0.0;  // Total acceleration magnitude (m/s²)
  float gyroX = 0.0;           // X-axis angular velocity (°/s)
  float gyroY = 0.0;           // Y-axis angular velocity (°/s)
  float gyroZ = 0.0;           // Z-axis angular velocity (°/s)

  // Environmental Data
  float temperature = 0.0;  // Temperature (°C)
  float pressure = 0.0;     // Atmospheric pressure (Pa)
  float altitude = 0.0;     // Calculated altitude (m)
};

/**
 * @brief Structure to hold system state information
 */
struct SystemState {
  float referencePressure =
      0.0;  // Ground level pressure for altitude calculation (Pa)
  bool sdCardReady = false;  // SD card initialization status
  bool buzzerArmed = false;  // Landing detection buzzer status
};

// ============================================================================
// GLOBAL SENSOR OBJECTS
// ============================================================================

extern SoftwareSerial imuSerial;  // Serial connection to IMU
extern DFRobot_WT61PC imuSensor;  // IMU sensor object
extern SensorReadings sensors;    // Current sensor readings
extern SystemState systemState;   // Current system state

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

/**
 * @brief Read temperature from LM35DT sensor
 * @param pin Analog pin connected to temperature sensor
 * @return Temperature in Celsius
 */
float readTemperature(int pin);

/**
 * @brief Read pressure from MPX4115A sensor
 * @param pin Analog pin connected to pressure sensor
 * @return Pressure in Pascals
 */
float readPressure(int pin);

/**
 * @brief Calculate altitude from pressure differential
 * @param currentPressure Current atmospheric pressure (Pa)
 * @param referencePressure Ground level pressure (Pa)
 * @return Altitude in meters
 */
float calculateAltitude(float currentPressure, float referencePressure);

/**
 * @brief Update all sensor readings
 */
void updateSensorReadings();

/**
 * @brief Check for landing condition and control buzzer
 */
void checkLandingCondition();

/**
 * @brief Initialize IMU sensor
 * @return true if successful, false otherwise
 */
bool initializeIMU();

/**
 * @brief Initialize reference pressure for altitude calculation
 */
void initializeReferencePressure();

#endif  // SENSORS_H