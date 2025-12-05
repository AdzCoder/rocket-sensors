# Testing Documentation

## Component Tests

### Accelerometer
- Verified with pendulum test
- Expected sine wave pattern observed
- Calibrated to read 0 at rest

### Pressure Sensor
- Tested in the pressure chamber
- Compared against a calibrated reference
- Accurate within 5% of reference values

### Temperature Sensor
- Showed response to heat changes
- Readings were erratic in the final assembly
- Suspected interference or connection issues

## System Tests

### Lift Test
- Simulated rocket flight in a building elevator
- Recorded:
  - Max height: 16.3m
  - Max acceleration: 0.74m/s²
  - Flight time: 19.8s
- Buzzer activated correctly when conditions were met

## Data Validation

Sample data from the lift test:
- Pressure changes correlated with height changes
- Acceleration spikes matched elevator movements
- Temperature data is unreliable
