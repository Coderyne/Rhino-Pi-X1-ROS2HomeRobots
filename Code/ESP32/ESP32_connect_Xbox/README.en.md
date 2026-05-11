# Xbox Series X Controller ESP32 Example

This project, built on the PlatformIO platform, enables BLE communication between an ESP32-C3 and an Xbox Series X controller.

## Project Overview

This project facilitates Bluetooth communication between an ESP32-C3 development board and an Xbox Series X / Xbox One controller. It parses input data such as buttons, joysticks, and triggers, outputs formatted data via serial, and supports vibration feedback.

## Hardware Requirements

- ESP32-C3 development board (default config: [weactstudio_esp32c3coreboard](https://github.com/WeActStudio/WeActStudio.ESP32C3CoreBoard))
- Xbox Series X / Xbox One controller (must support Bluetooth)

## Software Dependencies

- [PlatformIO](https://platformio.org/)
- Arduino framework
- [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino) (BLE stack)

## Project Structure

```
├── platformio.ini                               # PlatformIO configuration
├── src/
│   ├── main.cpp                                 # Main program entry
│   ├── XboxControllerNotificationParser.h       # Notification data parser
│   ├── XboxControllerNotificationParser.cpp     # Parser implementation
│   ├── XboxSeriesXControllerESP32_asukiaaa.hpp  # Controller BLE driver
│   └── XboxSeriesXHIDReportBuilder_asukiaaa.hpp  # HID vibration report builder
└── README.md
```

## Features

- Parse all button inputs (Y/X/B/A/LB/RB/Select/Start/Xbox/Share/LS/RS/D-pad)
- Read dual joystick analog values (normalized to -100~100)
- Read dual trigger values (normalized to 0~100)
- Four vibration feedback modes (left/right/center/shake) with adjustable power, duration, and repeat count
- Custom 11-byte binary protocol over Serial1 (GPIO 20/21) with header and checksum
- Auto-trigger 250ms vibration when receiving a `0x07 0x21` header frame on Serial1 (with 300ms debounce cooldown)
- Auto-reconnect on BLE disconnect; auto-restart after 2 failed connection attempts

## Usage

### 1. Configure Controller MAC Address

Edit `src/main.cpp` line 7, replacing with your controller's Bluetooth MAC address:

```cpp
XboxSeriesXControllerESP32_asukiaaa::Core
    xboxController("58:d0:05:0e:85:8d"); // Replace with your controller's address
```

> **Finding your controller's MAC address**: Windows Settings → Bluetooth & devices → Devices → Xbox Wireless Controller → Properties. Or check the serial log after scanning.

### 2. Compile and Upload

```bash
# Compile the project
pio run

# Upload to ESP32-C3
pio run --target upload

# Monitor serial output
pio device monitor
```

> PlatformIO installs dependencies automatically based on `platformio.ini`. No manual installation needed.

For other ESP32 boards, modify the `board` setting in `platformio.ini`.

### 3. Serial Monitor

After connecting the ESP32, the serial output will show connection status. Once connected, 11-byte binary data frames are continuously output via Serial1 (GPIO 20/21, 115200).

## Serial Data Protocol

After connection, an 11-byte binary frame is sent every 10ms via Serial1:

| Byte | Content |
|------|---------|
| 0-1 | Header `0x07 0x21` |
| 2 | Left joystick X (-100~100 mapped to 28~228) |
| 3 | Left joystick Y (-100~100 mapped to 28~228) |
| 4 | Right joystick X (-100~100 mapped to 28~228) |
| 5 | Right joystick Y (-100~100 mapped to 28~228) |
| 6 | Left trigger LT (0~100) |
| 7 | Right trigger RT (0~100) |
| 8 | Button group 1 (bit0:A, bit1:LB, bit2:RB, bit3:Xbox) |
| 9 | Button group 2 (bit0:B, bit1:X, bit2:Y) |
| 10 | Checksum (lower 8 bits of sum of first 10 bytes) |

## Vibration Control

Four vibration modes, freely combinable:

| Mode | Description |
|------|-------------|
| `left` | Upper-left motor |
| `right` | Upper-right motor |
| `center` | Lower dual motors, high frequency, low power |
| `shake` | Lower dual motors, low frequency, high power |

### Auto Vibration

When Serial1 receives a frame with header `0x07 0x21`, the controller automatically triggers a 250ms vibration (shake + center mode, 50% power) with a 300ms debounce cooldown. Useful for haptic feedback notifications from a remote device.

## Notes

1. You must set your controller's Bluetooth MAC address in the code before use
2. The controller must be in pairing mode (hold the pairing button on top, Xbox light flashes rapidly)
3. When using other ESP32 boards, adjust `board` and serial pin settings in `platformio.ini`
4. Lower-left and lower-right motors are bound together and can only operate simultaneously

