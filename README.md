# Rocket Instrumentation System

[![Arduino](https://img.shields.io/badge/Arduino-Micro-blue?style=flat-square)](https://www.arduino.cc/en/hardware/micro)
[![PlatformIO](https://img.shields.io/badge/PlatformIO-CI-orange?style=flat-square)](https://platformio.org/)
[![Licence](https://img.shields.io/badge/Licence-MIT-orange?style=flat-square)](LICENSE)
[![University](https://img.shields.io/badge/University-Warwick-green?style=flat-square)](https://warwick.ac.uk/)
[![Status](https://img.shields.io/badge/Status-Educational-lightgrey?style=flat-square)](https://github.com/topics/education)

Arduino payload firmware for a water rocket: it logs pressure, temperature, acceleration, and angular velocity to an SD card during flight, computes altitude on board, and sounds a buzzer after landing to help recover the payload. Built for a first-year electronics design sprint at Warwick; this repository holds the firmware, refactored from the original coursework sketch into a modular PlatformIO project.

## Background

This was the ES192 Engineering Design electronics sprint (2022/23), a five-person brief to design and build a payload sensor circuit for a water rocket. The circuit had to record atmospheric pressure, acceleration, and temperature to an SD card, then use that data to work out the rocket's maximum acceleration, peak height, and total flight time, alongside a visible on/off indicator and an audible locator. Adverse weather meant the payload was never flown; instead each sensor path was validated on the bench with a pendulum, a pressurised box, a heat gun, and a lift. Those tests showed the pressure and acceleration channels tracked well, while the temperature channel was the weakest link. This repository is the instrumentation firmware from that project.

## How it works

On start-up the firmware records a reference pressure, then loops: read the IMU, temperature, and pressure sensors, derive altitude from the pressure ratio against the reference, and append a timestamped row to the SD card. A landing is inferred when total acceleration sits near 1 g for a sustained window after an arming delay, which triggers the recovery buzzer. Each sensor read is range-checked, so a bad reading falls back to the last good value rather than corrupting the log.

## Hardware

| Sensor | Model | Measurement | Range |
|--------|-------|-------------|-------|
| IMU | [DFRobot WT61PC](https://www.dfrobot.com/product-2200.html) | 3-axis acceleration and angular velocity | ±16 g, ±2000°/s |
| Pressure | MPX4115A | Atmospheric pressure | 15–115 kPa |
| Temperature | LM35DT | Ambient temperature | −55 to +150 °C |

- **Microcontroller:** Arduino Micro (ATmega32U4)
- **Storage:** microSD card, FAT32
- **Power:** 9 V battery (6–12 V input)
- **Recovery aid:** piezo buzzer (~85 dB @ 10 cm)
- **Sampling:** roughly 10 Hz, set by the loop delay and IMU output rate

![Circuit plan](docs/circuit_plan.png)

## Build and upload

Requires [PlatformIO](https://platformio.org/). Dependencies are declared in `platformio.ini`.

```bash
git clone https://github.com/AdzCoder/rocket-sensors.git
cd rocket-sensors
pio run                    # build
pio run --target upload    # flash to the Micro
pio device monitor         # optional serial output
```

| Library | Purpose |
|---------|---------|
| [DFRobot_WT61PC](https://github.com/DFRobot/DFRobot_WT61PC) | IMU interface |
| SD | microSD logging |

## Data output

Rows are written to `flight_data.txt` on the card as CSV:

```
Time(ms),Pressure(Pa),Temperature(C),Height(m),AccelX,AccelY,AccelZ,TotalAccel(m/s2),GyroX,GyroY,GyroZ(deg/s)
```

The log supports post-flight analysis of flight phases, peak altitude and acceleration, and descent behaviour.

## Repository layout

- `src/`: `main.cpp` plus `sensors` and `data_logger` modules
- `include/`: `config.h` (pins, calibration, thresholds) and module headers
- `docs/`: `DESIGN.md`, `TESTING.md`, and circuit plan
- `data/`: bench-test logs
- `platformio.ini`: board and dependency configuration

## Academic context

**Module:** [ES192 Engineering Design (2022/23)](https://courses.warwick.ac.uk/modules/2022/ES192-15), Term 3 Electronics Design Sprint · **Team:** Group A04 · **Institution:** University of Warwick, School of Engineering

The project is complete and provided as an educational reference. Feel free to fork and adapt it.

## Licence

MIT Licence: see the [LICENCE](LICENSE) file for details.

---

*Developed by Adil Wahab Bhatti as part of academic coursework at the University of Warwick.*
