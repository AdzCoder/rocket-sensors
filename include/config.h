#ifndef CONFIG_H
#define CONFIG_H

#include <DFRobot_WT61PC.h>

// ============================================================================
// DEBUG CONFIGURATION
// ============================================================================

constexpr bool DEBUG_ENABLED = false;  // Set to true for serial debugging

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

namespace Pins {
constexpr int SD_CHIP_SELECT = 17;  // SD card CS pin
constexpr int IMU_RX = 10;          // IMU RXD connection
constexpr int IMU_TX = 11;          // IMU TXD connection
constexpr int TEMPERATURE = A3;     // LM35DT temperature sensor
constexpr int PRESSURE = A2;        // MPX4115A pressure sensor
constexpr int BUZZER = 13;          // Landing detection buzzer
}  // namespace Pins

// ============================================================================
// SYSTEM CONFIGURATION
// ============================================================================

namespace Config {
constexpr unsigned long BUZZER_ARM_TIME = 15000;  // Time to arm buzzer (ms)
constexpr unsigned long LOOP_DELAY = 200;         // Main loop delay (ms)
constexpr unsigned long LANDING_CONFIRM_TIME =
    2000;                                    // Landing confirmation time (ms)
constexpr unsigned long IMU_TIMEOUT = 3000;  // IMU initialization timeout (ms)
constexpr char DATA_FILENAME[] = "flight_data.txt";  // SD card data file
constexpr float LANDING_ACC_MIN =
    9.0;  // Minimum acceleration for landing (m/s²)
constexpr float LANDING_ACC_MAX =
    12.0;  // Maximum acceleration for landing (m/s²)
constexpr int IMU_FREQUENCY = FREQUENCY_5HZ;  // Data output frequency
}  // namespace Config

// ============================================================================
// SENSOR CALIBRATION CONSTANTS
// ============================================================================

namespace Calibration {
constexpr float TEMP_VOLTAGE_REF =
    5.0;  // Reference voltage for temperature sensor
constexpr float TEMP_SCALE_FACTOR =
    100.0;  // LM35DT conversion factor: 100°C per volt (from 10mV/°C)
constexpr float PRESSURE_OFFSET = 0.095;    // MPX4115A offset
constexpr float PRESSURE_SCALE = 0.000009;  // MPX4115A scale factor
constexpr float PRESSURE_MIN = 15000.0;  // MPX4115A minimum valid pressure (Pa)
constexpr float PRESSURE_MAX =
    115000.0;  // MPX4115A maximum valid pressure (Pa)
constexpr float ALTITUDE_EXPONENT = 1.0 / 5.257;  // Barometric formula exponent
constexpr float REFERENCE_TEMP_K =
    15.0 + 273.15;  // Reference temperature in Kelvin
}  // namespace Calibration

#endif  // CONFIG_H