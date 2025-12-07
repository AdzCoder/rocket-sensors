## Design Requirements

- Sampling rate: 10 Hz (100ms intervals)
- Flight duration: 15+ minutes on single 9V battery
- Temperature range: -10°C to +85°C operational
- Acceleration range: ±16g
- Storage: Minimum 100MB SD card

## Software Architecture

### Module Structure

- `main.cpp`: System initialization and main control loop
- `sensors.cpp/h`: Sensor data acquisition and processing
- `data_logger.cpp/h`: SD card file operations
- `config.h`: Pin definitions, calibration constants, system parameters

### Data Flow

1. IMU sensor polled via SoftwareSerial (5Hz output from sensor)
2. Analogue sensors read every 100ms
3. Altitude calculated from pressure using barometric formula
4. Data buffered and written to SD card
5. Landing detection monitors total acceleration magnitude

## Altitude Calculation Algorithm

Uses barometric formula: h = 44330 × (1 - (P/P₀)^(1/5.257))

- P₀: Reference pressure at ground level (measured at startup)
- P: Current atmospheric pressure
- Temperature compensation: Uses 15°C standard atmosphere

## Pin Assignments

- IMU (WT61PC): Pins 10(RX), 11(TX) - SoftwareSerial
- Pressure (MPX4115A): A2 (analogue)
- Temperature (LM35DT): A3 (analogue)
- SD Card: Pin 17 (CS), SPI bus
- Buzzer: Pin 13 (shared with onboard LED)

## Power Management

- Input: 9V battery (6-12V range supported)
- Arduino Micro onboard regulation to 5V
- Estimated current draw: ~150mA continuous
- Battery life: 15-20 minutes typical

## Error Handling Strategy

- SD card failure: Continue logging to serial, disable file writes
- IMU communication timeout: Skip IMU readings, continue with pressure/temp
- Sensor out-of-range values: Flag as invalid, continue operation
- Max consecutive failures tracked for diagnostics

## Components

[Keep existing Components section]
