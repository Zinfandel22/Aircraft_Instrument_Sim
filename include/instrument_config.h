#pragma once

#include <Arduino.h>

namespace aircraft {

constexpr int16_t kDisplayWidth = 466;
constexpr int16_t kDisplayHeight = 466;

constexpr uint8_t kDisplayCsPin = 12;
constexpr uint8_t kDisplayClockPin = 38;
constexpr uint8_t kDisplayData0Pin = 4;
constexpr uint8_t kDisplayData1Pin = 5;
constexpr uint8_t kDisplayData2Pin = 6;
constexpr uint8_t kDisplayData3Pin = 7;
constexpr uint8_t kDisplayResetPin = 39;

constexpr uint8_t kI2cSdaPin = 15;
constexpr uint8_t kI2cSclPin = 14;
constexpr uint8_t kTouchResetPin = 40;
constexpr uint8_t kTouchInterruptPin = 11;
constexpr uint8_t kTouchAddress = 0x5A;
constexpr uint8_t kImuAddress = 0x6B;
constexpr uint8_t kPmicAddress = 0x34;

}  // namespace aircraft
