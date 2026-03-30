# Parking Assist System

Reverse-activated ultrasonic parking assist system with adaptive acoustic warning and filtered distance measurement.

---

## Overview

This project implements a simplified automotive-style parking assist system using an ultrasonic sensor and a buzzer.

The system continuously measures the distance behind the vehicle and provides acoustic feedback to the driver. The warning frequency increases as the distance to an obstacle decreases, mimicking real-world parking assist systems.

The project is being developed as a rapid prototype on Arduino Nano, with a long-term goal of migrating to STM32 and extending the system with more advanced automotive-style features.

---

## Features

- UART-based ultrasonic distance measurement (A02YYUW)
- Real-time distance processing
- Distance-based adaptive acoustic warning
- Passive buzzer audio output using non-blocking tone control
- Audio layer prepared for future multi-sensor and fault-specific sound patterns
- Moving average filtering for stable distance measurement
- Hysteresis-based distance zones for stable behavior
- Reverse activation simulated using push button input
- State machine-based control flow (IDLE / ACTIVE / FAULT)
- Non-blocking timing using `millis()`
- Packet validation using checksum
- Timeout-based safety handling
- Structured fault detection framework (extensible for multiple fault types)

---

## Hardware

- Arduino Nano
- Waterproof ultrasonic sensor (A02YYUW / A0221AU)
- Passive buzzer
- Push button (reverse activation simulation)
- Breadboard and jumper wires

---

## Wiring

### Ultrasonic Sensor (UART)

| Sensor Wire | Arduino |
|------------|--------|
| Red (VCC)  | 5V     |
| Black (GND)| GND    |
| Blue (TX)  | D10    |
| Green (RX) | D11    |

### Buzzer

| Buzzer Pin | Arduino |
|-----------|--------|
| +         | D3     |
| -         | GND    |

### Reverse Simulation Button

| Button Pin | Arduino |
|-----------|--------|
| 1 side    | D2     |
| 1 side    | GND    |

---

## System Behavior

The system operates in real time:

- The sensor continuously sends distance data via UART
- The Arduino parses incoming packets and validates them using checksum
- Distance is calculated and mapped to warning zones
- The passive buzzer generates acoustic feedback based on proximity
- The system is active only when reverse input is enabled

### Warning Zones

| Distance (cm) | Behavior        |
|--------------|----------------|
| > 100        | No sound       |
| 60–100       | Slow beeping   |
| 40–60        | Medium beeping |
| 20–40        | Fast beeping   |
| < 20         | Continuous tone|

---

## Control Logic

The system is only active when reverse gear is engaged.

For development and testing purposes, reverse activation is simulated using a push button input (active LOW).

When reverse is not active:
- Sensor data is ignored
- Audio output is disabled

---

## Software Architecture

The system is structured into three logical parts:

- **Data acquisition** – reading UART packets from the sensor  
- **Processing** – validating, filtering and converting distance data  
- **Decision logic** – warning zone selection with hysteresis  
- **Audio output** – passive buzzer control using non-blocking timing
- **State control** – system-level mode handling using a state machine

This structure is intended to support future expansion with:
- multiple ultrasonic sensors
- fault-specific warning patterns
- state machine control
- migration to STM32

---

## Communication Protocol

The sensor sends 4-byte packets:

```
[0] 0xFF -> Start byte
[1] High byte -> Distance (MSB)
[2] Low byte -> Distance (LSB)
[3] Checksum -> (byte0 + byte1 + byte2) & 0xFF
```


Distance is converted to centimeters:
```
distance = ((high_byte << 8) + low_byte) / 10
```

---

## Safety Features

- **Checksum validation** ensures data integrity  
- **Timeout mechanism** disables audio output if sensor data is lost  
- **Reverse-controlled operation** prevents continuous system activity outside reverse mode

---

## Current Status

**V4 – State machine-based system control**

- Distance measurement 
- Filtering
- Hysteresis 
- Passive buzzer audio control
- Reverse activation
- State machine control

---

## Future Improvements
  
- Sensor fault detection  
- Dirty sensor error detection
- Fault-specific acoustic warning patterns
- Additional ultrasonic sensor for curb detection
- IMU-based tilt-aware warning logic
- Migration from Arduino Nano to STM32
- CAN-based vehicle communication

---

## Disclaimer

This project is for educational and prototyping purposes only and is not intended for use in safety-critical automotive systems.

---

## License

This project is licensed under the MIT License.