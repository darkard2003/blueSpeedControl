# ESP32 Bluetooth Motor Control

This project implements a Bluetooth-controlled motor driver system using an ESP32 microcontroller. It allows wireless control of a two-motor (left/right) robot or vehicle using differential steering.

## Features

- Bluetooth connectivity with automatic reconnection
- Differential steering control (speed + direction)
- PWM motor control
- LED status indication
- Compatible with standard L298N or similar H-bridge motor drivers

## Hardware Requirements

- ESP32 development board (ESP32-DOIT DevKit v1)
- Dual motor driver (like L298N)
- DC motors (2x)
- Power supply appropriate for your motors
- Bluetooth-enabled device for control (smartphone, tablet, etc.)

## Pin Configuration

### Left Motor
- IN1: GPIO 16
- IN2: GPIO 17
- EN: GPIO 4 (PWM capable)

### Right Motor
- IN1: GPIO 5
- IN2: GPIO 18
- EN: GPIO 15 (PWM capable)

## Communication Protocol

The system expects a 2-byte control sequence over Bluetooth:
- Byte 1: Speed value (0-255, with 127 being neutral/stop)
- Byte 2: Steering value (0-255, with 127 being straight)

## Development Environment

This project is built using PlatformIO with the Arduino framework for ESP32.

## Building and Uploading

1. Install [PlatformIO](https://platformio.org/)
2. Clone this repository
3. Open the project in PlatformIO
4. Connect your ESP32 via USB
5. Build and upload the firmware

## License

This project is licensed under the GNU General Public License v3.0 - see the [LICENSE](LICENSE) file for details.