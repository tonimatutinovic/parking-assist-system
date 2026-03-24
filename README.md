# Parking Assist System

Reverse-activated ultrasonic parking assist system with adaptive acoustic warning and filtered distance measurement.

---

## Overview

This project implements a simplified automotive-style parking assist system using an ultrasonic sensor and a buzzer.

The system continuously measures the distance behind the vehicle and provides acoustic feedback to the driver. The warning frequency increases as the distance to an obstacle decreases, mimicking real-world parking assist systems.

---

## Features

- UART-based ultrasonic distance measurement (A02YYUW)
- Real-time distance processing
- Distance-based adaptive acoustic warning
- Non-blocking timing using `millis()`
- Packet validation using checksum
- Timeout-based safety handling
- Moving average filtering for stable distance measurement
- Hysteresis-based distance zones for stable behavior
- Active buzzer for acoustic warning (ON/OFF control)
- Reverse activation simulated using push button input

---

## Hardware

- Arduino Nano
- Waterproof ultrasonic sensor (A02YYUW / A0221AU)
- Buzzer (active)
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

---

## System Behavior

The system operates in real time:

- The sensor continuously sends distance data via UART
- The Arduino parses incoming packets and validates them using checksum
- Distance is calculated and mapped to warning zones
- The buzzer generates acoustic feedback based on proximity

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
- Buzzer is disabled

---

## Software Architecture

The system is structured into three logical parts:

- **Data acquisition** – reading UART packets from the sensor  
- **Processing** – validating and converting distance data  
- **Actuation** – controlling the buzzer using non-blocking timing  

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
- **Timeout mechanism** disables buzzer if sensor data is lost  

---

## Current Status

**V2 – Reverse activation and improved control logic**

- Distance measurement 
- Filtering
- Hysteresis 
- Active buzzer
- Reverse activation

---

## Future Improvements

- Reverse gear activation input  
- Sensor fault detection  
- State machine architecture  
- Multi-sensor support  

---

## Disclaimer

This project is for educational and prototyping purposes only and is not intended for use in safety-critical automotive systems.

---

## License

This project is licensed under the MIT License.