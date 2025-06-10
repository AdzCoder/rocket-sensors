# Rocket Instrumentation System

An Arduino-based payload sensor system designed for collecting telemetry data during water rocket flights.

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
    Follow the hardware setup instructions in [`DESIGN.md`](docs/DESIGN.md).

4. **Upload to Arduino Micro**  
    Once hardware is connected and libraries are installed, upload the project to your Arduino Micro.

## Data Format

Data is logged to the SD card in CSV format with the following columns:

- **Time** (ms)  
- **Pressure** (Pa)  
- **Temperature** (°C)  
- **Height** (m)  
- **Acceleration X** (m/s²)  
- **Acceleration Y** (m/s²)  
- **Acceleration Z** (m/s²)  
- **Total Acceleration** (m/s²)  
- **Angular Velocity X** (wX)  
- **Angular Velocity Y** (wY)  
- **Angular Velocity Z** (wZ)  

## Project Team

- A04  
- University of Warwick, School of Engineering  
- ES192 2022/2023 Sprint 2 - Electronic

## Project Status

This project was completed as part of coursework for the University of Warwick (ES192 module, 2022/2023).  
It is no longer actively maintained, but the code and documentation are provided for reference and learning purposes.

## License

MIT License — see the [LICENSE](LICENSE) file for details.
