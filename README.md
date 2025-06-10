# Rocket Instrumentation System

An Arduino-based payload sensor system designed for collecting telemetry data during water rocket flights.

## Features

- Triaxial acceleration measurement using the [DFRobot WT61PC accelerometer](https://github.com/DFRobot/DFRobot_WT61PC)
- Atmospheric pressure measurement with the [MPX4115A pressure sensor](https://github.com/janlucaklees/MPX4115A-Arduino-library)
- Temperature measurement via the [LM35DT temperature sensor](https://github.com/Erriez/ErriezLM35)
- Data logging to microSD card
- Audible recovery buzzer
- Visual power indicator

## Hardware Requirements

- Arduino Micro
- [DFRobot WT61PC accelerometer](https://www.dfrobot.com/product-2200.html)
- MPX4115A pressure sensor
- LM35DT temperature sensor
- MicroSD card breakout board
- Piezo buzzer
- LED indicator
- 9V battery

## Installation

1. **Clone this repository**
    ```bash
    git clone https://github.com/AdzCoder/rocket-sensor.git
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

- Group A04  
- University of Warwick, School of Engineering  
- [ES192: Engineering Design (22/23)](https://courses.warwick.ac.uk/modules/2022/ES192-15) - Electronic Sprint 2

## Project Status

This project was completed as part of coursework for the University of Warwick (ES192 module, 2022/2023).  
It is no longer actively maintained, but the code and documentation are provided for reference and learning purposes.

## License

MIT License — see the [LICENSE](LICENSE) file for details.
