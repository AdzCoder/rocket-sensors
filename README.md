# Rocket Instrumentation System

An Arduino-based payload sensor system for water rocket telemetry data collection.

## Features

- Triaxial acceleration measurement (DFRobot WT61PC accelerometer)
- Atmospheric pressure measurement (MPX4115A pressure sensor)
- Temperature measurement (LM35DT sensor)
- Data logging to microSD card
- Audible recovery buzzer
- Visual power indicator

## Hardware Requirements

- Arduino Micro
- DFRobot WT61PC accelerometer
- MPX4115A pressure sensor
- LM35DT temperature sensor
- MicroSD card breakout board
- Piezo buzzer
- LED indicator
- 9V battery

## Installation

1. Clone this repository
2. Open `src/RocketSensors.ino` in Arduino IDE
3. Connect hardware as specified in DESIGN.md
4. Upload to Arduino Micro

## Data Format

Data is logged to SD card in CSV format with columns:
Time (ms), Pressure (Pa), Temp (°C), Height (m), AccX (m/s²), AccY (m/s²), AccZ (m/s²), AccTotal (m/s²), wX, wY, wZ

## License

MIT License - see LICENSE file
