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
- Altitude is a pitch-derived estimate at 100 ft/min per degree of pitch (positive pitch increases the estimate); connect a barometer or aircraft telemetry for measured altitude

## Variometer
- The QMI8658 pitch angle is converted to a rate command at 100 ft/min per degree. Positive rates indicate climb; negative rates indicate descent.
- Positive rate samples are averaged over the latest eight updates before reaching the audio task. The audio task applies additional low-pass smoothing to avoid abrupt cue changes.
- Rates above 50 ft/min produce climb beeps. Beep pitch rises from 500 to 1,400 Hz and the interval shortens from 900 to 250 ms as the filtered rate approaches 3,000 ft/min. Each 100 ms beep completes at its starting pitch; rate changes affect the next beep.
- Rates below -50 ft/min produce a continuous 260 Hz descent tone. Rates within the deadband are silent.
- Audio uses the ES8311 codec and the ESP32-S3 I2S output at 16 kHz, 16-bit stereo. A short 1 kHz tone plays at startup to verify the speaker path.
- This is a pitch-driven estimate, not measured aircraft vertical speed. Use a barometer or aircraft telemetry for a true variometer.

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
