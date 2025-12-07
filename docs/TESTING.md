# Testing Documentation

## Test Methodology

### Unit Tests

#### 1. IMU Sensor (DFRobot WT61PC)

**Test Procedure:**

1. Mount sensor on pendulum apparatus
2. Record acceleration data during controlled swings
3. Compare against theoretical sine wave pattern

**Results:**

- Sensor output frequency: 5 Hz confirmed
- Acceleration range: ±16g verified
- Noise level: <0.1 m/s² at rest
- ✓ PASSED

**Data:** See `data/acceleration_test_data.csv`

#### 2. Pressure Sensor (MPX4115A)

**Test Procedure:**

1. Place sensor in calibrated pressure chamber
2. Vary pressure from 15-115 kPa in 10 kPa steps
3. Compare readings against reference barometer

**Results:**

- Accuracy: Within ±5% of reference (±5 kPa)
- Linearity: Good across operating range
- Response time: <50ms
- ✓ PASSED with acceptable tolerance

**Data:** See `data/pressure_test_data.csv`

#### 3. Temperature Sensor (LM35DT)

**Test Procedure:**

1. Apply heat source at controlled distances
2. Monitor sensor response time and accuracy
3. Compare against calibrated thermocouple

**Results:**

- Response time: ~5 seconds to 63% of final value
- Accuracy: ±2°C when isolated
- **Issue:** Erratic readings in final assembly
- **Diagnosis:** Suspected electromagnetic interference from SD card SPI bus or thermal coupling to processor
- **Status:** ⚠ FAILED - Needs shielding/relocation

**Data:** See `data/heat_test_data.csv`

### Integration Tests

#### System Integration Test

**Test Procedure:**

1. Assemble complete payload
2. Power from 9V battery
3. Run continuous logging for 20 minutes
4. Verify all subsystems operational

**Results:**

- Power consumption: Stable at ~145mA
- SD write reliability: 100% success rate
- Data integrity: All records complete
- ✓ PASSED

#### Lift Test (Elevator Simulation)

**Test Setup:**

- Location: Engineering building elevator
- Simulates vertical rocket trajectory
- Controlled environment with known altitude changes

**Test Procedure:**

1. Start system on ground floor
2. Ride elevator to top floor (height: 16.3m measured)
3. Descend back to ground
4. Verify buzzer activation on landing

**Results:**

| Parameter            | Measured Value | Notes                               |
| -------------------- | -------------- | ----------------------------------- |
| Maximum height       | 16.3m          | Matches building specs              |
| Maximum acceleration | 0.74 m/s²      | Elevator acceleration               |
| Flight duration      | 19.8s          | Ascent only                         |
| Buzzer activation    | ✓ Success      | Activated on landing after 2s delay |
| Data points captured | 198            | At 10 Hz sampling                   |

**Data:** See `data/lift_test_data.csv`

**Observations:**

- Pressure-derived altitude accurate within 0.5m
- Acceleration spikes correctly captured during start/stop
- Temperature data unreliable (see Unit Test #3)

### Landing Detection Algorithm Test

**Test Conditions:**

- Total acceleration between 9.0-12.0 m/s² (simulates at-rest condition)
- Must be sustained for 2 seconds minimum
- Buzzer arms after 15 seconds of operation

**Results:**

- ✓ Correctly ignored transient accelerations
- ✓ Activated after confirmed landing state
- ✓ No false positives during 5 test runs

## Known Issues

1. **Temperature Sensor Interference**
   - Impact: Temperature data unreliable
   - Workaround: Use pressure sensor temperature compensation
   - Future fix: Relocate sensor away from high-frequency digital lines

2. **SD Card Initialization Delay**
   - Impact: 2-3 second startup delay
   - Mitigation: Acceptable for pre-launch preparation time

## Validation Summary

| Component          | Status             | Confidence Level |
| ------------------ | ------------------ | ---------------- |
| IMU                | ✓ Operational      | High             |
| Pressure Sensor    | ✓ Operational      | High             |
| Temperature Sensor | ⚠ Degraded         | Low              |
| SD Logging         | ✓ Operational      | High             |
| Buzzer Recovery    | ✓ Operational      | High             |
| **Overall System** | **✓ Flight Ready** | **Medium-High**  |

## Recommendations

1. **Pre-flight**: Verify SD card write with serial monitor
2. **Temperature**: Disable or treat as informational only
3. **Calibration**: Record reference pressure at launch site
4. **Recovery**: Test buzzer before flight
5. **Future**: Add redundant storage or wireless telemetry
