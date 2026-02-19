# Wiring and Power Design

## System Overview

The smart car uses:

- ESP32 Dev Board
- H-Bridge Motor Driver
- Active Buzzer (5V via transistor)
- RGB LED
- Voltage Regulator

---

## Power Architecture

### 3.3V Rail
Used for:
- ESP32 logic
- PS5 Bluetooth stack

### 5V Rail
Used for:
- Active buzzer (through transistor)

Why?

The buzzer volume was too low at 3.3V.
Driving it from 5V via a transistor ensures:
- Loud output
- No ESP32 pin overload
- Stable operation

---

## Buzzer Wiring

- ESP32 pin → Base of BC547 (via resistor)
- Collector → Buzzer negative
- Buzzer positive → 5V
- Emitter → GND

This allows:
- ESP32 to control buzzer safely
- Buzzer to run at full 5V
- No resets due to power noise

---

## Motor Driver Pins

| ESP32 Pin | Function |
|-----------|----------|
| 5  | M1A |
| 18 | M1B |
| 19 | M2A |
| 21 | M2B |

PWM frequency: 10 kHz  
Resolution: 8-bit (0–255)

---

## Ground Considerations

All grounds must be common between:
- ESP32
- Motor driver
- Voltage regulator
- Buzzer transistor

Incorrect grounding may cause:
- Random resets
- Unstable Bluetooth
- Buzzer glitches
