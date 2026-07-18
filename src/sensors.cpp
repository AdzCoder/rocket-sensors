#include "sensors.h"

#include "config.h"

// Global sensor objects (actual definitions)
SoftwareSerial imuSerial(Pins::IMU_RX, Pins::IMU_TX);
DFRobot_WT61PC imuSensor(&imuSerial);
SensorReadings sensors;
SystemState systemState;

/**
 * @brief Read temperature from LM35DT sensor with validation
 * @param pin Analog pin connected to temperature sensor
 * @return Temperature in Celsius
 */
float readTemperature(int pin) {
  float rawReading = analogRead(pin);
  float voltage = (rawReading / 1023.0) * Calibration::TEMP_VOLTAGE_REF;
  float temperature = voltage * Calibration::TEMP_SCALE_FACTOR;

  // Validate temperature reading (LM35DT valid range: -55°C to +150°C)
  if (temperature < Calibration::TEMP_MIN ||
      temperature > Calibration::TEMP_MAX) {
    systemState.temperatureFailureCount++;
    if (DEBUG_ENABLED &&
        systemState.temperatureFailureCount == Config::MAX_SENSOR_FAILURES) {
      Serial.println(
          F("⚠ WARNING: Temperature sensor failures exceeded threshold"));
    }
    return systemState.referenceTemperature;  // Return last known good value
  }

  // Reset failure counter and update reference on successful read
  systemState.temperatureFailureCount = 0;
  systemState.referenceTemperature = temperature;
  return temperature;
}

/**
 * @brief Read pressure from MPX4115A sensor with validation
 * @param pin Analog pin connected to pressure sensor
 * @return Pressure in Pascals
 */
float readPressure(int pin) {
  int rawReading = analogRead(pin);
  // Convert ADC reading to voltage using the shared reference
  float voltage = (rawReading / 1023.0) * Calibration::TEMP_VOLTAGE_REF;

  // MPX4115A: Vout = VS × (0.009 × P - 0.095) where P is in kPa
  // Rearranged: P = (Vout/VS + 0.095) / 0.009
  float pressureKPa =
      (voltage / Calibration::TEMP_VOLTAGE_REF + Calibration::PRESSURE_OFFSET) /
      Calibration::PRESSURE_SCALE;
  float pressure = pressureKPa * 1000.0;  // Convert kPa to Pa

  // Validate pressure reading (MPX4115A valid range: 15-115 kPa)
  if (pressure < Calibration::PRESSURE_MIN ||
      pressure > Calibration::PRESSURE_MAX) {
    systemState.pressureFailureCount++;
    if (DEBUG_ENABLED &&
        systemState.pressureFailureCount == Config::MAX_SENSOR_FAILURES) {
      Serial.println(
          F("⚠ WARNING: Pressure sensor failures exceeded threshold"));
    }
    return systemState.referencePressure;  // Return last known good value
  }

  // Reset failure counter on successful read
  systemState.pressureFailureCount = 0;
  return pressure;
}

/**
 * @brief Calculate altitude from pressure differential
 * @param currentPressure Current atmospheric pressure (Pa)
 * @param referencePressure Ground level pressure (Pa)
 * @return Altitude in meters
 */
float calculateAltitude(float currentPressure, float referencePressure) {
  if (referencePressure == 0.0 || currentPressure == 0.0) return 0.0;

  float pressureRatio = referencePressure / currentPressure;

  // Guard against non-physical pressure ratios
  if (pressureRatio <= 0.0) return 0.0;

  float altitudeFactor =
      pow(pressureRatio, Calibration::ALTITUDE_EXPONENT) - 1.0;
  return altitudeFactor * Calibration::REFERENCE_TEMP_K /
         0.0065;  // Standard lapse rate
}

/**
 * @brief Update all sensor readings
 */
void updateSensorReadings() {
  sensors.timestamp = millis();

  // Read IMU data if available
  if (systemState.imuAvailable && imuSensor.available()) {
    sensors.accelX = imuSensor.Acc.X;
    sensors.accelY = imuSensor.Acc.Y;
    sensors.accelZ = imuSensor.Acc.Z;
    sensors.accelMagnitude =
        sqrt(pow(sensors.accelX, 2) + pow(sensors.accelY, 2) +
             pow(sensors.accelZ, 2));
    sensors.gyroX = imuSensor.Gyro.X;
    sensors.gyroY = imuSensor.Gyro.Y;
    sensors.gyroZ = imuSensor.Gyro.Z;
  }

  // Read environmental sensors
  sensors.temperature = readTemperature(Pins::TEMPERATURE);
  sensors.pressure = readPressure(Pins::PRESSURE);
  sensors.altitude =
      calculateAltitude(sensors.pressure, systemState.referencePressure);
}

/**
 * @brief Check for landing condition and control buzzer with debouncing
 */
void checkLandingCondition() {
  static unsigned long landingStartTime = 0;
  systemState.buzzerArmed = (sensors.timestamp >= Config::BUZZER_ARM_TIME);

  bool inLandingRange = systemState.buzzerArmed &&
                        (sensors.accelMagnitude >= Config::LANDING_ACC_MIN) &&
                        (sensors.accelMagnitude <= Config::LANDING_ACC_MAX);

  if (inLandingRange) {
    if (landingStartTime == 0) {
      landingStartTime = millis();
      // Unsigned subtraction below is overflow-safe
    } else if ((unsigned long)(millis() - landingStartTime) >=
               Config::LANDING_CONFIRM_TIME) {
      digitalWrite(Pins::BUZZER, HIGH);
    }
  } else {
    landingStartTime = 0;
    digitalWrite(Pins::BUZZER, LOW);
  }
}

/**
 * @brief Initialize IMU sensor
 * @return true if successful, false otherwise
 */
bool initializeIMU() {
  imuSerial.begin(9600);
  imuSensor.modifyFrequency(Config::IMU_FREQUENCY);

  // Wait for IMU to respond with timeout
  unsigned long imuTimeout = millis() + Config::IMU_TIMEOUT;
  while (!imuSensor.available() && millis() < imuTimeout) {
    delay(10);
  }

  systemState.imuAvailable = imuSensor.available();

  if (DEBUG_ENABLED) {
    if (systemState.imuAvailable) {
      Serial.println(F("✓ IMU sensor initialized"));
    } else {
      Serial.println(
          F("⚠ WARNING: IMU not responding - continuing without IMU data"));
    }

    Serial.print(F("✓ Data frequency set to "));
    switch (Config::IMU_FREQUENCY) {
      case FREQUENCY_5HZ:
        Serial.println(F("5 Hz"));
        break;
      case FREQUENCY_10HZ:
        Serial.println(F("10 Hz"));
        break;
      case FREQUENCY_20HZ:
        Serial.println(F("20 Hz"));
        break;
      default:
        Serial.println(F("Unknown"));
        break;
    }
  }

  return systemState.imuAvailable;
}

/**
 * @brief Initialize reference pressure for altitude calculation
 */
void initializeReferencePressure() {
  delay(1000);  // Allow sensors to stabilize

  // Validate the reading before storing it as the ground reference
  float pressure = readPressure(Pins::PRESSURE);

  // Only store if within valid range
  if (pressure >= Calibration::PRESSURE_MIN &&
      pressure <= Calibration::PRESSURE_MAX) {
    systemState.referencePressure = pressure;
  } else {
    // Use standard sea level pressure as fallback
    systemState.referencePressure = 101325.0;  // Standard atmosphere in Pa
    if (DEBUG_ENABLED) {
      Serial.println(F(
          "⚠ WARNING: Invalid reference pressure, using standard atmosphere"));
    }
  }

  if (DEBUG_ENABLED) {
    Serial.print(F("✓ Reference pressure: "));
    Serial.print(systemState.referencePressure, 1);
    Serial.println(F(" Pa"));
  }
}