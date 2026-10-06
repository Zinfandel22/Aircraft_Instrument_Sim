# Aircraft Instruments

This project targets the Waveshare ESP32-S3-Touch-AMOLED-1.75 (standard, product 31261):
- ESP32-S3R8, dual-core Xtensa LX7 up to 240 MHz
- 16 MB flash and 8 MB OPI PSRAM
- 466x466 CO5300 AMOLED with QSPI interface
- CST9217 capacitive touch and QMI8658 six-axis IMU
- AXP2101 power-management IC

## Current setup
- PlatformIO board: `esp32-s3-devkitc-1`
- QIO flash and OPI PSRAM configured for the Waveshare module
- CO5300 QSPI display driven by Arduino_GFX
- CST9217 touch, QMI8658 IMU, and AXP2101 PMIC share I2C on GPIO 15/14
- Panel brightness is controlled by the CO5300 driver, not a backlight GPIO

## Features
- ESP32-S3 hardware profile configured
- Attitude, compass, altimeter, and airplane display modes
- Touch input cycles through the instrument screens
- Attitude pitch and roll use raw QMI8658 accelerometer X/Y/Z; +X near 1 g defines level
- Compass heading is gyro-relative and will drift because this board has no magnetometer
- Altitude is a pitch-derived estimate at 100 ft/min per degree of pitch (nose-up descends); connect a barometer or aircraft telemetry for measured altitude

## Getting started
1. Install PlatformIO and the VS Code PlatformIO extension.
2. Open this folder in VS Code.
3. Build the project:
   ```bash
   pio run
   ```
4. Upload to the board:
   ```bash
   pio run -t upload
   ```
5. Open the serial monitor:
   ```bash
   pio device monitor
   ```

## Customization
- Waveshare display, touch, IMU, and PMIC pin mappings are in `include/instrument_config.h` and `src/main.cpp`.
- Expand `src/main.cpp` with gauges, sensor reads, and touchscreen interaction.
- Add modules for engine, battery, and navigation instrumentation as the project grows.
