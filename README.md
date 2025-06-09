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

1. **Clone this repository**
    ```bash
    git clone <repository-url>
    ```

2. **Open the project**  
    Open `src/RocketSensors.ino` using the [Arduino IDE](https://www.arduino.cc/en/software).

3. **Connect hardware**  
    Follow the hardware setup instructions in [`DESIGN.md`](DESIGN.md).

4. **Upload to Arduino Micro**  
    Once hardware is connected and libraries are installed, upload the project to your Arduino Micro.

## Data Format

Data is logged to SD card in CSV format with columns:
Time (ms), Pressure (Pa), Temp (°C), Height (m), AccX (m/s²), AccY (m/s²), AccZ (m/s²), AccTotal (m/s²), wX, wY, wZ

## Project Team
- A04
- University of Warwick, School of Engineering
- ES192 2022/2023 Sprint 2 - Electronic

## License

MIT License - see LICENSE file
