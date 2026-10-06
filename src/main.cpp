#include <Arduino_GFX_Library.h>
#include <ImuDrv.hpp>
#include <XPowersLib.h>
#include <esp_heap_caps.h>
#include <Wire.h>

#include "instrument_config.h"

namespace {

constexpr int kDisplayWidth = aircraft::kDisplayWidth;
constexpr int kDisplayHeight = aircraft::kDisplayHeight;
constexpr int kCenterX = kDisplayWidth / 2;
constexpr int kCenterY = kDisplayHeight / 2;
constexpr float kArtworkScale = static_cast<float>(kDisplayWidth) / 480.0f;

Arduino_DataBus *g_panelBus = new Arduino_ESP32QSPI(
    aircraft::kDisplayCsPin,
    aircraft::kDisplayClockPin,
    aircraft::kDisplayData0Pin,
    aircraft::kDisplayData1Pin,
    aircraft::kDisplayData2Pin,
    aircraft::kDisplayData3Pin);

Arduino_CO5300 *g_display = new Arduino_CO5300(
    g_panelBus, aircraft::kDisplayResetPin, 0,
    kDisplayWidth, kDisplayHeight, 6, 0, 0, 0);

class PsramCanvas : public Arduino_Canvas
{
public:
    explicit PsramCanvas(Arduino_G *output)
        : Arduino_Canvas(kDisplayWidth, kDisplayHeight, output)
    {
    }

    bool begin(int32_t speed = GFX_NOT_DEFINED) override
    {
        if (!_output->begin(speed)) {
            return false;
        }
        _framebuffer = static_cast<uint16_t *>(heap_caps_malloc(
            static_cast<size_t>(kDisplayWidth) * kDisplayHeight * sizeof(uint16_t),
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
        return _framebuffer != nullptr;
    }
};

PsramCanvas *g_tft = new PsramCanvas(g_display);
XPowersAXP2101 g_pmic;
SensorQMI8658 g_imu;
bool g_imuReady = false;

constexpr uint16_t kBlack = 0x0000;
constexpr uint16_t kWhite = 0xFFFF;
constexpr uint16_t kSky = 0x153C;
constexpr uint16_t kGround = 0x92E5;
constexpr uint16_t kOrange = 0xFD20;
constexpr uint16_t kCyan = 0x07FF;
constexpr uint16_t kDarkGray = 0x2965;
constexpr uint16_t kLightGray = 0xAD55;
constexpr uint16_t kNavy = 0x000F;
constexpr uint16_t kRingEdge = 0x0861;
constexpr uint16_t kYellow = 0xFFE0;
constexpr int kBarometricSetting = 1024;

enum ScreenMode {
    SCREEN_ATTITUDE = 0,
    SCREEN_COMPASS,
    SCREEN_ALTIMETER,
    SCREEN_AIRPLANE,
    SCREEN_COUNT
};

ScreenMode g_currentScreen = SCREEN_ATTITUDE;
float g_pitchDeg = 0.0f;
float g_rollDeg = 0.0f;
float g_turnCoordinatorRollDeg = 0.0f;
float g_headingDeg = 0.0f;
float g_altitudeFt = 6300.0f;
float g_lateralAccelerationMps2 = 0.0f;
uint32_t g_lastImuUpdateMs = 0;
float g_gravityX = 0.0f;
float g_gravityY = 0.0f;
float g_gravityZ = 0.0f;
bool g_gravityInitialized = false;
uint32_t g_lastImuLogMs = 0;

void drawCenteredText(const String &text, int x, int y, uint8_t size, uint16_t color, uint16_t background)
{
    g_tft->setTextSize(size);
    g_tft->setTextColor(color, background);
    const int textWidth = text.length() * 6 * size;
    g_tft->setCursor(x - textWidth / 2, y);
    g_tft->print(text);
}

void drawNumberText(int value, int x, int y, uint8_t size, uint16_t color, uint16_t background)
{
    g_tft->setTextSize(size);
    g_tft->setTextColor(color, background);
    g_tft->setCursor(x, y);
    g_tft->print(value);
}

void resetTouchController()
{
    pinMode(aircraft::kTouchResetPin, OUTPUT);
    digitalWrite(aircraft::kTouchResetPin, LOW);
    delay(10);
    digitalWrite(aircraft::kTouchResetPin, HIGH);
    delay(50);
}

bool initializePmic()
{
    if (!g_pmic.begin(Wire, aircraft::kPmicAddress,
                      aircraft::kI2cSdaPin, aircraft::kI2cSclPin)) {
        return false;
    }

    g_pmic.setALDO1Voltage(1800);
    g_pmic.enableALDO1();
    g_pmic.setALDO2Voltage(1800);
    g_pmic.enableALDO2();
    g_pmic.setALDO3Voltage(3300);
    g_pmic.enableALDO3();
    delay(250);
    return true;
}

bool initializeImu()
{
    if (!g_imu.begin(Wire, QMI8658_L_SLAVE_ADDRESS,
                     aircraft::kI2cSdaPin, aircraft::kI2cSclPin)) {
        return false;
    }

    const bool accelConfigured = g_imu.configAccel(
        AccelFullScaleRange::FS_4G, 448.0f, SensorQMI8658::LpfMode::MODE_0);
    const bool gyroConfigured = g_imu.configGyro(
        GyroFullScaleRange::FS_500_DPS, 448.0f, SensorQMI8658::LpfMode::MODE_0);
    return accelConfigured && gyroConfigured && g_imu.enableAccel() && g_imu.enableGyro();
}

float wrapAngleDegrees(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

bool readTouch(uint16_t &x, uint16_t &y)
{
    Wire.beginTransmission(aircraft::kTouchAddress);
    Wire.write(0xD0);
    Wire.write(0x00);
    if (Wire.endTransmission(true) != 0) {
        return false;
    }
    delayMicroseconds(200);

    uint8_t data[15] = {};
    const uint8_t received = Wire.requestFrom(
        aircraft::kTouchAddress, static_cast<uint8_t>(sizeof(data)));
    if (received < sizeof(data)) {
        while (Wire.available()) {
            Wire.read();
        }
        return false;
    }
    for (uint8_t &value : data) {
        value = static_cast<uint8_t>(Wire.read());
    }

    Wire.beginTransmission(aircraft::kTouchAddress);
    Wire.write(0xD0);
    Wire.write(0x00);
    Wire.write(0xAB);
    Wire.endTransmission(true);

    if (data[0] == 0xAB || data[6] != 0xAB || (data[5] & 0x7F) == 0 ||
        (data[0] & 0x0F) != 0x06) {
        return false;
    }

    x = static_cast<uint16_t>((static_cast<uint16_t>(data[1]) << 4) | (data[3] >> 4));
    y = static_cast<uint16_t>((static_cast<uint16_t>(data[2]) << 4) | (data[3] & 0x0F));
    return x < kDisplayWidth && y < kDisplayHeight;
}

void updateImu()
{
    if (!g_imuReady ||
        !g_imu.isDataReady(static_cast<uint8_t>(ImuBase::DataReadyMask::ACCEL))) {
        return;
    }

    AccelerometerData accel = {};
    GyroscopeData gyro = {};
    if (!g_imu.readAccel(accel) || !g_imu.readGyro(gyro)) {
        return;
    }

    const uint32_t now = millis();
    const float dt = (g_lastImuUpdateMs == 0)
        ? 0.02f
        : constrain((now - g_lastImuUpdateMs) / 1000.0f, 0.001f, 0.1f);
    g_lastImuUpdateMs = now;

    const float ax = accel.mps2.x;
    const float ay = accel.mps2.y;
    const float az = accel.mps2.z;
    g_lateralAccelerationMps2 = ay - g_gravityY;
    const float gravityAlpha = 1.0f - expf(-dt / 0.8f);
    if (!g_gravityInitialized) {
        g_gravityX = ax;
        g_gravityY = ay;
        g_gravityZ = az;
        g_gravityInitialized = true;
    } else {
        g_gravityX += gravityAlpha * (ax - g_gravityX);
        g_gravityY += gravityAlpha * (ay - g_gravityY);
        g_gravityZ += gravityAlpha * (az - g_gravityZ);
    }

    if (now - g_lastImuLogMs >= 250) {
        g_lastImuLogMs = now;
        Serial.printf(
            "[IMU] Gravity X=%+.2f Y=%+.2f Z=%+.2f m/s^2 | Accel x'=%+.2f y'=%+.2f z'=%+.2f m/s^2\n",
            g_gravityX, g_gravityY, g_gravityZ,
            ax - g_gravityX, ay - g_gravityY, az - g_gravityZ);
    }

    const float accelRoll = wrapAngleDegrees(atan2f(-ay, ax) * 180.0f / PI);
    const float accelPitch = atan2f(az, sqrtf(ax * ax + ay * ay)) * 180.0f / PI;

    g_rollDeg = accelRoll;
    g_pitchDeg = accelPitch;
    const float bankDelta = wrapAngleDegrees(g_rollDeg - g_turnCoordinatorRollDeg);
    const float bankAlpha = 1.0f - expf(-dt / 0.18f);
    g_turnCoordinatorRollDeg = wrapAngleDegrees(g_turnCoordinatorRollDeg + bankAlpha * bankDelta);
    g_headingDeg = fmodf(g_headingDeg + gyro.dps.z * dt + 360.0f, 360.0f);
}

void updateAltitudeFromPitch()
{
    static uint32_t previousUpdateMs = 0;
    const uint32_t now = millis();
    if (previousUpdateMs == 0) {
        previousUpdateMs = now;
        return;
    }

    const uint32_t elapsedMs = now - previousUpdateMs;
    previousUpdateMs = now;

    const float verticalSpeedFtPerMinute = -g_pitchDeg * 100.0f;

    g_altitudeFt += verticalSpeedFtPerMinute * (elapsedMs / 60000.0f);
    g_altitudeFt = constrain(g_altitudeFt, 0.0f, 99999.0f);
}

uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

void drawLine(int x0, int y0, int x1, int y1, uint16_t color)
{
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        if ((x0 >= 0) && (x0 < kDisplayWidth) && (y0 >= 0) && (y0 < kDisplayHeight)) {
            g_tft->drawPixel(x0, y0, color);
        }
        if ((x0 == x1) && (y0 == y1)) {
            break;
        }
        const int e2 = err * 2;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void drawThickLine(int x0, int y0, int x1, int y1, uint16_t color, int thickness)
{
    const float deltaX = static_cast<float>(x1 - x0);
    const float deltaY = static_cast<float>(y1 - y0);
    const float length = sqrtf(deltaX * deltaX + deltaY * deltaY);
    if (length == 0.0f) {
        g_tft->fillCircle(x0, y0, max(1, thickness / 2), color);
        return;
    }

    const float normalX = -deltaY / length;
    const float normalY = deltaX / length;
    for (int offset = -thickness / 2; offset <= thickness / 2; ++offset) {
        const int offsetX = static_cast<int>(lroundf(normalX * offset));
        const int offsetY = static_cast<int>(lroundf(normalY * offset));
        drawLine(x0 + offsetX, y0 + offsetY, x1 + offsetX, y1 + offsetY, color);
    }
}

void drawCircle(int cx, int cy, int radius, uint16_t color)
{
    int x = radius;
    int y = 0;
    int err = 0;

    while (x >= y) {
        g_tft->drawPixel(cx + x, cy + y, color);
        g_tft->drawPixel(cx + y, cy + x, color);
        g_tft->drawPixel(cx - y, cy + x, color);
        g_tft->drawPixel(cx - x, cy + y, color);
        g_tft->drawPixel(cx - x, cy - y, color);
        g_tft->drawPixel(cx - y, cy - x, color);
        g_tft->drawPixel(cx + y, cy - x, color);
        g_tft->drawPixel(cx + x, cy - y, color);

        if (err <= 0) {
            y += 1;
            err += 2 * y + 1;
        }
        if (err > 0) {
            x -= 1;
            err -= 2 * x + 1;
        }
    }
}

void fillCircle(int cx, int cy, int radius, uint16_t color)
{
    g_tft->fillCircle(cx, cy, radius, color);
}

void fillTriangle(int x1, int y1, int x2, int y2, int x3, int y3, uint16_t color)
{
    const int minX = min(x1, min(x2, x3));
    const int maxX = max(x1, max(x2, x3));
    const int minY = min(y1, min(y2, y3));
    const int maxY = max(y1, max(y2, y3));

    for (int y = minY; y <= maxY; ++y) {
        for (int x = minX; x <= maxX; ++x) {
            const int w1 = (x2 - x1) * (y - y1) - (y2 - y1) * (x - x1);
            const int w2 = (x3 - x2) * (y - y2) - (y3 - y2) * (x - x2);
            const int w3 = (x1 - x3) * (y - y3) - (y1 - y3) * (x - x3);
            const bool inside = ((w1 >= 0 && w2 >= 0 && w3 >= 0) || (w1 <= 0 && w2 <= 0 && w3 <= 0));
            if (inside) {
                g_tft->drawPixel(x, y, color);
            }
        }
    }
}

void fillRing(int cx, int cy, int innerRadius, int outerRadius, uint16_t color)
{
    for (int y = -outerRadius; y <= outerRadius; ++y) {
        for (int x = -outerRadius; x <= outerRadius; ++x) {
            const int d2 = x * x + y * y;
            if (d2 >= innerRadius * innerRadius && d2 <= outerRadius * outerRadius) {
                g_tft->drawPixel(cx + x, cy + y, color);
            }
        }
    }
}

void drawSegDigit(int d, int x, int y, int scale, uint16_t color)
{
    bool seg[7] = {false};

    switch (d) {
        case 0: seg[0] = true; seg[1] = true; seg[2] = true; seg[3] = true; seg[4] = true; seg[5] = true; break;
        case 1: seg[1] = true; seg[2] = true; break;
        case 2: seg[0] = true; seg[1] = true; seg[3] = true; seg[4] = true; seg[6] = true; break;
        case 3: seg[0] = true; seg[1] = true; seg[2] = true; seg[3] = true; seg[6] = true; break;
        case 4: seg[1] = true; seg[2] = true; seg[5] = true; seg[6] = true; break;
        case 5: seg[0] = true; seg[2] = true; seg[3] = true; seg[5] = true; seg[6] = true; break;
        case 6: seg[0] = true; seg[2] = true; seg[3] = true; seg[4] = true; seg[5] = true; seg[6] = true; break;
        case 7: seg[0] = true; seg[1] = true; seg[2] = true; break;
        case 8: seg[0] = true; seg[1] = true; seg[2] = true; seg[3] = true; seg[4] = true; seg[5] = true; seg[6] = true; break;
        case 9: seg[0] = true; seg[1] = true; seg[2] = true; seg[3] = true; seg[5] = true; seg[6] = true; break;
        default: break;
    }

    const int w = 5 * scale;
    const int h = 9 * scale;
    const int thickness = scale;

    if (seg[0]) drawThickLine(x, y, x + w, y, color, thickness);
    if (seg[1]) drawThickLine(x + w, y, x + w, y + h / 2, color, thickness);
    if (seg[2]) drawThickLine(x + w, y + h / 2, x + w, y + h, color, thickness);
    if (seg[3]) drawThickLine(x, y + h, x + w, y + h, color, thickness);
    if (seg[4]) drawThickLine(x, y + h / 2, x, y + h, color, thickness);
    if (seg[5]) drawThickLine(x, y, x, y + h / 2, color, thickness);
    if (seg[6]) drawThickLine(x, y + h / 2, x + w, y + h / 2, color, thickness);
}

void drawSmallNumber(int value, int x, int y, uint16_t color)
{
    if (value == 10) {
        drawSegDigit(1, x, y, 2, color);
        drawSegDigit(0, x + 14, y, 2, color);
    }
    if (value == 20) {
        drawSegDigit(2, x, y, 2, color);
        drawSegDigit(0, x + 14, y, 2, color);
    }
}

int scaleArtwork(float value)
{
    return static_cast<int>(lroundf(value * kArtworkScale));
}

void transformAttitudePoint(float localX, float localY, int &x, int &y)
{
    const float rollRadians = g_rollDeg * PI / 180.0f;
    const float cosine = cosf(rollRadians);
    const float sine = sinf(rollRadians);
    x = kCenterX + scaleArtwork(localX * cosine + localY * sine);
    y = kCenterY + scaleArtwork(-localX * sine + localY * cosine);
}

void drawAttitudeLine(float x0, float y0, float x1, float y1, uint16_t color, int thickness)
{
    int startX;
    int startY;
    int endX;
    int endY;
    transformAttitudePoint(x0, y0, startX, startY);
    transformAttitudePoint(x1, y1, endX, endY);
    drawThickLine(startX, startY, endX, endY, color, thickness);
}

void drawAttitudeBezel()
{
    fillRing(kCenterX, kCenterY, scaleArtwork(207), scaleArtwork(224), kDarkGray);
    fillRing(kCenterX, kCenterY, scaleArtwork(225), scaleArtwork(239), kLightGray);
    drawCircle(kCenterX, kCenterY, scaleArtwork(206), kRingEdge);
    drawCircle(kCenterX, kCenterY, scaleArtwork(224), kRingEdge);
    drawCircle(kCenterX, kCenterY, scaleArtwork(239), kWhite);
}

void drawArtificialHorizon()
{
    constexpr int logicalRadius = 206;
    const int radius = scaleArtwork(logicalRadius);
    const float rollRadians = g_rollDeg * PI / 180.0f;
    const float cosine = cosf(rollRadians);
    const float sine = sinf(rollRadians);
    const float pitchOffset = -g_pitchDeg * 4.0f;

    fillCircle(kCenterX, kCenterY, radius, kSky);
    for (int y = -radius; y <= radius; ++y) {
        const int xExtent = static_cast<int>(sqrtf(radius * radius - y * y));
        for (int x = -xExtent; x <= xExtent; ++x) {
            const float logicalX = x / kArtworkScale;
            const float logicalY = y / kArtworkScale;
            const float horizonY = logicalX * sine + logicalY * cosine + pitchOffset;
            if (horizonY >= 0.0f) {
                g_tft->drawPixel(kCenterX + x, kCenterY + y, kGround);
            }
        }
    }

    int x1;
    int y1;
    int x2;
    int y2;
    int x3;
    int y3;
    transformAttitudePoint(0, 10 - pitchOffset, x1, y1);
    transformAttitudePoint(-95, 115 - pitchOffset, x2, y2);
    transformAttitudePoint(95, 115 - pitchOffset, x3, y3);
    fillTriangle(x1, y1, x2, y2, x3, y3, rgb565(80, 38, 25));

    drawAttitudeLine(0, -pitchOffset, 0, 10 - pitchOffset, kWhite, 3);
    drawAttitudeLine(-95, 115 - pitchOffset, -32, 115 - pitchOffset, kOrange, 2);
    drawAttitudeLine(32, 115 - pitchOffset, 95, 115 - pitchOffset, kOrange, 2);
    drawAttitudeLine(-32, 115 - pitchOffset, 32, 115 - pitchOffset, kOrange, 2);
    drawAttitudeLine(-logicalRadius, -pitchOffset, logicalRadius, -pitchOffset, kWhite, 3);

    for (int mark = -4; mark <= 4; ++mark) {
        if (mark == 0) {
            continue;
        }
        const int value = abs(mark) * 5;
        const int halfLength = (value == 10 || value == 20) ? 60 : 29;
        const float y = mark * 32.0f - pitchOffset;
        drawAttitudeLine(-halfLength, y, halfLength, y, kWhite, 3);

        if (value == 10 || value == 20) {
            int leftX;
            int leftY;
            int rightX;
            int rightY;
            transformAttitudePoint(-halfLength - 50, y - 12, leftX, leftY);
            transformAttitudePoint(halfLength + 24, y - 12, rightX, rightY);
            drawSmallNumber(value, leftX, leftY, kWhite);
            drawSmallNumber(value, rightX, rightY, kWhite);
        }
    }
}

void drawAircraftSymbol()
{
    drawThickLine(kCenterX - scaleArtwork(140), kCenterY,
                  kCenterX - scaleArtwork(35), kCenterY, kYellow, scaleArtwork(8));
    drawThickLine(kCenterX + scaleArtwork(35), kCenterY,
                  kCenterX + scaleArtwork(140), kCenterY, kYellow, scaleArtwork(8));
    drawThickLine(kCenterX - scaleArtwork(35), kCenterY,
                  kCenterX, kCenterY + scaleArtwork(28), kYellow, scaleArtwork(8));
    drawThickLine(kCenterX, kCenterY + scaleArtwork(28),
                  kCenterX + scaleArtwork(35), kCenterY, kYellow, scaleArtwork(8));
    fillTriangle(kCenterX, kCenterY - scaleArtwork(4),
                 kCenterX - scaleArtwork(7), kCenterY + scaleArtwork(6),
                 kCenterX + scaleArtwork(7), kCenterY + scaleArtwork(6), kYellow);
}

void drawRollScale()
{
    for (int angle = -50; angle <= 50; angle += 10) {
        const float radians = (angle - 90.0f) * PI / 180.0f;
        const int innerRadius = scaleArtwork(angle % 30 == 0 ? 207 : 211);
        const int outerRadius = scaleArtwork(224);
        drawThickLine(
            kCenterX + static_cast<int>(cosf(radians) * innerRadius),
            kCenterY + static_cast<int>(sinf(radians) * innerRadius),
            kCenterX + static_cast<int>(cosf(radians) * outerRadius),
            kCenterY + static_cast<int>(sinf(radians) * outerRadius),
            kWhite, scaleArtwork(3));
    }

    fillTriangle(kCenterX, kCenterY - scaleArtwork(210),
                 kCenterX - scaleArtwork(11), kCenterY - scaleArtwork(229),
                 kCenterX + scaleArtwork(11), kCenterY - scaleArtwork(229), kOrange);

    int tipX;
    int tipY;
    int leftX;
    int leftY;
    int rightX;
    int rightY;
    transformAttitudePoint(0, -200, tipX, tipY);
    transformAttitudePoint(-12, -181, leftX, leftY);
    transformAttitudePoint(12, -181, rightX, rightY);
    fillTriangle(tipX, tipY, leftX, leftY, rightX, rightY, kWhite);
}

void drawFrame()
{
    g_tft->drawRoundRect(20, 20, kDisplayWidth - 40, kDisplayHeight - 40, 26, kDarkGray);
    g_tft->drawCircle(kCenterX, kCenterY, 218, kLightGray);
    g_tft->drawCircle(kCenterX, kCenterY, 228, kDarkGray);
}

void drawCompassTicks(float headingDeg)
{
    for (int deg = 0; deg < 360; deg += 10) {
        const float angle = (deg - headingDeg - 90.0f) * PI / 180.0f;
        const int outerRadius = scaleArtwork(216);
        const int innerRadius = scaleArtwork((deg % 30 == 0) ? 184 : 196);
        const int x0 = kCenterX + static_cast<int>(cosf(angle) * innerRadius);
        const int y0 = kCenterY + static_cast<int>(sinf(angle) * innerRadius);
        const int x1 = kCenterX + static_cast<int>(cosf(angle) * outerRadius);
        const int y1 = kCenterY + static_cast<int>(sinf(angle) * outerRadius);
        drawThickLine(x0, y0, x1, y1, kWhite, scaleArtwork((deg % 30 == 0) ? 4 : 2));
    }

    for (int deg = 5; deg < 360; deg += 10) {
        const float angle = (deg - headingDeg - 90.0f) * PI / 180.0f;
        const int radius = scaleArtwork(195);
        const int x = kCenterX + static_cast<int>(cosf(angle) * radius);
        const int y = kCenterY + static_cast<int>(sinf(angle) * radius);
        fillCircle(x, y, scaleArtwork(2), kWhite);
    }
}

void drawCompassLetters(float headingDeg)
{
    struct CompassMark {
        int degrees;
        const char *label;
        int radius;
        uint8_t textSize;
        uint16_t color;
    };
    static constexpr CompassMark marks[] = {
        {0, "N", 158, 3, kYellow}, {30, "3", 160, 2, kWhite},
        {60, "6", 160, 2, kWhite}, {90, "E", 158, 3, kYellow},
        {120, "12", 160, 2, kWhite}, {150, "15", 165, 2, kWhite},
        {180, "S", 158, 3, kYellow}, {210, "21", 155, 2, kWhite},
        {240, "24", 150, 2, kWhite}, {270, "W", 158, 3, kYellow},
        {300, "30", 150, 2, kWhite}, {330, "33", 150, 2, kWhite}
    };

    for (const CompassMark &mark : marks) {
        const float angle = (mark.degrees - headingDeg - 90.0f) * PI / 180.0f;
        const int radius = scaleArtwork(static_cast<float>(mark.radius));
        const int x = kCenterX + static_cast<int>(cosf(angle) * radius);
        const int y = kCenterY + static_cast<int>(sinf(angle) * radius);
        drawCenteredText(mark.label, x, y - 4 * mark.textSize,
                         mark.textSize, mark.color, kBlack);
    }
}

void drawCompassAircraft()
{
    const uint16_t color = kOrange;
    auto coordinate = [](float value) { return scaleArtwork(value * 0.8f); };
    auto line = [&](float x0, float y0, float x1, float y1, int thickness) {
        drawThickLine(kCenterX + coordinate(x0), kCenterY + coordinate(y0),
                      kCenterX + coordinate(x1), kCenterY + coordinate(y1),
                      color, scaleArtwork(static_cast<float>(thickness)));
    };

    line(0, -194, 0, 65, 5);
    line(0, -130, -28, -40, 4);
    line(0, -130, 28, -40, 4);
    line(-28, -40, -88, 10, 4);
    line(28, -40, 88, 10, 4);
    line(-88, 10, -88, 35, 4);
    line(88, 10, 88, 35, 4);
    line(-88, 35, -20, 10, 4);
    line(88, 35, 20, 10, 4);
    line(-20, 10, -20, 85, 4);
    line(20, 10, 20, 85, 4);
    line(-20, 85, -55, 110, 4);
    line(20, 85, 55, 110, 4);
    line(-55, 110, -55, 130, 4);
    line(55, 110, 55, 130, 4);
    line(-55, 130, 0, 108, 4);
    line(55, 130, 0, 108, 4);
}

void drawCompassScreen()
{
    g_tft->fillScreen(kBlack);
    fillCircle(kCenterX, kCenterY, scaleArtwork(239), kDarkGray);
    fillCircle(kCenterX, kCenterY, scaleArtwork(228), kRingEdge);
    fillCircle(kCenterX, kCenterY, scaleArtwork(218), kBlack);
    drawCompassTicks(g_headingDeg);
    drawCompassLetters(g_headingDeg);
    drawCompassAircraft();
    drawCircle(kCenterX, kCenterY, scaleArtwork(217), kLightGray);
    drawCircle(kCenterX, kCenterY, scaleArtwork(228), kDarkGray);
    drawCircle(kCenterX, kCenterY, scaleArtwork(239), kWhite);
}

void drawAltimeterTicks()
{
    const int radius = scaleArtwork(211);
    for (int tick = 0; tick < 100; ++tick) {
        const float angle = (tick * 3.6f - 90.0f) * PI / 180.0f;
        const bool major = (tick % 10 == 0);
        const bool medium = (tick % 5 == 0);
        const int innerRadius = scaleArtwork(major ? 177 : (medium ? 185 : 193));
        const int x0 = kCenterX + static_cast<int>(cosf(angle) * innerRadius);
        const int y0 = kCenterY + static_cast<int>(sinf(angle) * innerRadius);
        const int x1 = kCenterX + static_cast<int>(cosf(angle) * radius);
        const int y1 = kCenterY + static_cast<int>(sinf(angle) * radius);
        drawThickLine(x0, y0, x1, y1, kWhite, major ? 4 : 2);
    }
}

void drawAltimeterNumbers()
{
    const int numberRadius = scaleArtwork(155);
    for (int number = 0; number < 10; ++number) {
        const float angle = (number * 36.0f - 90.0f) * PI / 180.0f;
        const int x = kCenterX + static_cast<int>(cosf(angle) * numberRadius);
        const int y = kCenterY + static_cast<int>(sinf(angle) * numberRadius);
        drawCenteredText(String(number), x, y - scaleArtwork(16), 4, kWhite, kBlack);
    }
}

void drawLongAltimeterHand(float angleDeg, int length, int halfWidth, uint16_t color)
{
    const float angle = (angleDeg - 90.0f) * PI / 180.0f;
    const int scaledLength = scaleArtwork(static_cast<float>(length));
    const int scaledWidth = scaleArtwork(static_cast<float>(halfWidth));
    const float directionX = cosf(angle);
    const float directionY = sinf(angle);
    const float normalX = -directionY;
    const float normalY = directionX;
    const int shaftStart = scaleArtwork(14);
    const int shaftEnd = scaledLength * 4 / 5;

    auto pointOnNeedle = [&](float distance, float width, int side, int &x, int &y) {
        x = kCenterX + static_cast<int>(lroundf(directionX * distance + normalX * width * side));
        y = kCenterY + static_cast<int>(lroundf(directionY * distance + normalY * width * side));
    };

    int baseLeftX, baseLeftY, baseRightX, baseRightY;
    int shaftLeftX, shaftLeftY, shaftRightX, shaftRightY;
    int tipX, tipY;
    pointOnNeedle(shaftStart, scaledWidth, -1, baseLeftX, baseLeftY);
    pointOnNeedle(shaftStart, scaledWidth, 1, baseRightX, baseRightY);
    pointOnNeedle(shaftEnd, scaledWidth, -1, shaftLeftX, shaftLeftY);
    pointOnNeedle(shaftEnd, scaledWidth, 1, shaftRightX, shaftRightY);
    pointOnNeedle(scaledLength, 0, 0, tipX, tipY);

    fillTriangle(baseLeftX, baseLeftY, baseRightX, baseRightY, shaftLeftX, shaftLeftY, color);
    fillTriangle(baseRightX, baseRightY, shaftLeftX, shaftLeftY, shaftRightX, shaftRightY, color);
    fillTriangle(shaftLeftX, shaftLeftY, shaftRightX, shaftRightY, tipX, tipY, color);
}

void drawDiamondAltimeterHand(float angleDeg, int length, int halfWidth, uint16_t color)
{
    const float angle = (angleDeg - 90.0f) * PI / 180.0f;
    const float directionX = cosf(angle);
    const float directionY = sinf(angle);
    const float normalX = -directionY;
    const float normalY = directionX;
    const int scaledLength = scaleArtwork(static_cast<float>(length));
    const int scaledWidth = scaleArtwork(static_cast<float>(halfWidth));
    const int pivotInset = scaleArtwork(14);
    const int widestPoint = scaledLength * 45 / 100;

    auto pointOnNeedle = [&](float distance, float width, int side, int &x, int &y) {
        x = kCenterX + static_cast<int>(lroundf(directionX * distance + normalX * width * side));
        y = kCenterY + static_cast<int>(lroundf(directionY * distance + normalY * width * side));
    };

    int baseX, baseY, shoulderLeftX, shoulderLeftY;
    int shoulderRightX, shoulderRightY, tipX, tipY;
    pointOnNeedle(pivotInset, 0, 0, baseX, baseY);
    pointOnNeedle(widestPoint, scaledWidth, -1, shoulderLeftX, shoulderLeftY);
    pointOnNeedle(widestPoint, scaledWidth, 1, shoulderRightX, shoulderRightY);
    pointOnNeedle(scaledLength, 0, 0, tipX, tipY);

    fillTriangle(baseX, baseY, shoulderLeftX, shoulderLeftY, tipX, tipY, color);
    fillTriangle(baseX, baseY, tipX, tipY, shoulderRightX, shoulderRightY, color);
}

void drawAltimeterScreen()
{
    g_tft->fillScreen(kBlack);

    fillRing(kCenterX, kCenterY, scaleArtwork(229), scaleArtwork(239), kLightGray);
    fillRing(kCenterX, kCenterY, scaleArtwork(218), scaleArtwork(228), kDarkGray);
    drawCircle(kCenterX, kCenterY, scaleArtwork(217), kRingEdge);
    drawCircle(kCenterX, kCenterY, scaleArtwork(228), kRingEdge);
    drawCircle(kCenterX, kCenterY, scaleArtwork(239), kWhite);
    fillCircle(kCenterX, kCenterY, scaleArtwork(217), kBlack);

    drawAltimeterTicks();
    drawAltimeterNumbers();
    drawCenteredText("1000 FEET", kCenterX, kCenterY - scaleArtwork(99), 1, kLightGray, kBlack);
    drawCenteredText("10000 FEET", kCenterX, kCenterY - scaleArtwork(78), 1, kLightGray, kBlack);
    drawCenteredText("ALT", kCenterX + scaleArtwork(51), kCenterY - scaleArtwork(10), 2, kWhite, kBlack);

    const int altitude = constrain(static_cast<int>(g_altitudeFt), 0, 99999);
    const float longAngle = (altitude % 1000) * 360.0f / 1000.0f;
    const float shortAngle = (altitude % 10000) * 360.0f / 10000.0f;
    drawDiamondAltimeterHand(shortAngle, 96, 12, kLightGray);
    drawLongAltimeterHand(longAngle, 174, 4, kWhite);
    fillCircle(kCenterX, kCenterY, scaleArtwork(14), kDarkGray);
    fillCircle(kCenterX, kCenterY, scaleArtwork(6), kWhite);

    const int windowWidth = scaleArtwork(142);
    const int windowHeight = scaleArtwork(42);
    const int windowX = kCenterX - windowWidth / 2;
    const int windowY = kCenterY + scaleArtwork(78);
    g_tft->fillRect(windowX, windowY, windowWidth, windowHeight, kBlack);
    g_tft->drawRect(windowX, windowY, windowWidth, windowHeight, kLightGray);
    const String settingText(kBarometricSetting);
    const int digitStride = scaleArtwork(21);
    const int digitsWidth = (settingText.length() - 1) * digitStride + scaleArtwork(10);
    int digitX = kCenterX - digitsWidth / 2;
    for (size_t index = 0; index < settingText.length(); ++index) {
        drawSegDigit(settingText[index] - '0', digitX, windowY + scaleArtwork(8), 2, rgb565(255, 58, 35));
        digitX += digitStride;
    }
}

void drawAttitudeScreen()
{
    g_tft->fillScreen(kBlack);
    drawAttitudeBezel();
    drawArtificialHorizon();
    drawAircraftSymbol();
    drawRollScale();
}

void drawAirplaneScreen()
{
    g_tft->fillScreen(kBlack);
    const int outerRadius = scaleArtwork(239);
    const int innerRadius = scaleArtwork(216);
    fillCircle(kCenterX, kCenterY, outerRadius, kDarkGray);
    fillCircle(kCenterX, kCenterY, scaleArtwork(229), kLightGray);
    fillCircle(kCenterX, kCenterY, scaleArtwork(222), kRingEdge);
    fillCircle(kCenterX, kCenterY, innerRadius, kBlack);
    drawCircle(kCenterX, kCenterY, outerRadius, kWhite);

    drawCenteredText("TURN COORDINATOR", kCenterX, kCenterY - scaleArtwork(132), 2, kWhite, kBlack);

    const int bankMarkY = kCenterY + scaleArtwork(4);
    const int bankMarkHalfWidth = scaleArtwork(13);
    const int bankMarkOffset = scaleArtwork(166);
    drawThickLine(kCenterX - bankMarkOffset - bankMarkHalfWidth, bankMarkY,
                  kCenterX - bankMarkOffset + bankMarkHalfWidth, bankMarkY, kWhite, scaleArtwork(5));
    drawThickLine(kCenterX + bankMarkOffset - bankMarkHalfWidth, bankMarkY,
                  kCenterX + bankMarkOffset + bankMarkHalfWidth, bankMarkY, kWhite, scaleArtwork(5));

    const int lowerMarkY = kCenterY + scaleArtwork(50);
    const int lowerMarkOffset = scaleArtwork(156);
    drawThickLine(kCenterX - lowerMarkOffset - scaleArtwork(12), lowerMarkY - scaleArtwork(5),
                  kCenterX - lowerMarkOffset + scaleArtwork(12), lowerMarkY + scaleArtwork(5), kWhite, scaleArtwork(7));
    drawThickLine(kCenterX + lowerMarkOffset - scaleArtwork(12), lowerMarkY + scaleArtwork(5),
                  kCenterX + lowerMarkOffset + scaleArtwork(12), lowerMarkY - scaleArtwork(5), kWhite, scaleArtwork(7));

    drawCenteredText("L", kCenterX - lowerMarkOffset, lowerMarkY + scaleArtwork(26), 2, kWhite, kBlack);
    drawCenteredText("R", kCenterX + lowerMarkOffset, lowerMarkY + scaleArtwork(26), 2, kWhite, kBlack);

    auto drawBankedAircraftLine = [&](float x0, float y0, float x1, float y1, int thickness) {
        constexpr float aircraftScale = 1.38f;
        const float angle = g_turnCoordinatorRollDeg * PI / 180.0f;
        const float cosine = cosf(angle);
        const float sine = sinf(angle);
        const int startX = kCenterX + scaleArtwork(aircraftScale * (x0 * cosine - y0 * sine));
        const int startY = kCenterY + scaleArtwork(aircraftScale * (x0 * sine + y0 * cosine));
        const int endX = kCenterX + scaleArtwork(aircraftScale * (x1 * cosine - y1 * sine));
        const int endY = kCenterY + scaleArtwork(aircraftScale * (x1 * sine + y1 * cosine));
        drawThickLine(startX, startY, endX, endY, kWhite,
                      scaleArtwork(static_cast<float>(thickness) * aircraftScale));
    };

    drawBankedAircraftLine(0, -61, 0, 42, 8);
    drawBankedAircraftLine(0, -5, -104, -11, 10);
    drawBankedAircraftLine(0, -5, 104, -11, 10);
    drawBankedAircraftLine(-104, -11, -78, -9, 6);
    drawBankedAircraftLine(104, -11, 78, -9, 6);
    drawBankedAircraftLine(-44, -19, 44, -19, 8);
    drawBankedAircraftLine(0, -61, 0, -8, 8);
    fillCircle(kCenterX - scaleArtwork(34), kCenterY - scaleArtwork(8), scaleArtwork(9), kWhite);
    fillCircle(kCenterX + scaleArtwork(34), kCenterY - scaleArtwork(8), scaleArtwork(9), kWhite);
    fillCircle(kCenterX, kCenterY, scaleArtwork(13), kWhite);

    const int tubeWidth = scaleArtwork(246);
    const int tubeHeight = scaleArtwork(58);
    const int tubeX = kCenterX - tubeWidth / 2;
    const int tubeY = kCenterY + scaleArtwork(62);
    const int tubeHalfHeight = tubeHeight / 2;
    const int tubeEndRadius = tubeHalfHeight;
    const int tubeStraightHalfWidth = tubeWidth / 2 - tubeEndRadius;
    const int tubeBowHeight = scaleArtwork(7);
    const uint16_t tubeColor = rgb565(190, 186, 157);

    auto tubeMidYAt = [&](int x) {
        const float normalizedX = static_cast<float>(x - kCenterX) / (tubeWidth / 2.0f);
        const float bow = tubeBowHeight * (1.0f - normalizedX * normalizedX);
        return tubeY + tubeHalfHeight + static_cast<int>(lroundf(bow));
    };

    int previousX = tubeX;
    int previousTopY = tubeMidYAt(tubeX);
    int previousBottomY = previousTopY;
    for (int x = tubeX; x <= tubeX + tubeWidth; ++x) {
        const int distanceFromCenter = abs(x - kCenterX);
        int halfSpan = tubeHalfHeight;
        if (distanceFromCenter > tubeStraightHalfWidth) {
            const int capOffset = distanceFromCenter - tubeStraightHalfWidth;
            const int capRadiusSquared = tubeEndRadius * tubeEndRadius;
            halfSpan = static_cast<int>(sqrtf(static_cast<float>(max(
                0, capRadiusSquared - capOffset * capOffset))));
        }

        const int middleY = tubeMidYAt(x);
        const int topY = middleY - halfSpan;
        const int bottomY = middleY + halfSpan;
        g_tft->drawFastVLine(x, topY, max(1, bottomY - topY + 1), tubeColor);
        if (x > tubeX) {
            drawLine(previousX, previousTopY, x, topY, kLightGray);
            drawLine(previousX, previousBottomY, x, bottomY, kLightGray);
        }
        previousX = x;
        previousTopY = topY;
        previousBottomY = bottomY;
    }

    const int tubeCenterY = tubeMidYAt(kCenterX);
    drawThickLine(kCenterX, tubeCenterY - scaleArtwork(22),
                  kCenterX, tubeCenterY + scaleArtwork(22), kRingEdge, 2);

    const int ballTravel = scaleArtwork(83);
    const int ballOffset = static_cast<int>(lroundf(
        constrain(g_lateralAccelerationMps2 / 4.0f, -1.0f, 1.0f) * ballTravel));
    const int ballX = kCenterX + ballOffset;
    const int ballY = tubeMidYAt(ballX);
    fillCircle(ballX, ballY, scaleArtwork(18), kBlack);
    drawCircle(ballX, ballY, scaleArtwork(18), kWhite);

    drawCenteredText("2 MIN.", kCenterX, tubeY + tubeHeight + scaleArtwork(18), 2, kWhite, kBlack);
    drawCenteredText("NO PITCH", kCenterX, tubeY + tubeHeight + scaleArtwork(47), 1, kLightGray, kBlack);
    drawCenteredText("INFORMATION", kCenterX, tubeY + tubeHeight + scaleArtwork(63), 1, kLightGray, kBlack);
    drawCircle(kCenterX, kCenterY, innerRadius, kRingEdge);
}

void renderBootScreen()
{
    g_tft->fillScreen(kBlack);
    drawCenteredText("AIRCRAFT INSTRUMENTS", kCenterX, 150, 3, kCyan, kBlack);
    drawCenteredText("Waveshare AMOLED", kCenterX, 210, 3, kOrange, kBlack);
    drawCenteredText("466x466", kCenterX, 250, 3, kWhite, kBlack);
    drawFrame();
    g_tft->flush();
}

void renderScreen()
{
    switch (g_currentScreen) {
        case SCREEN_ATTITUDE:
            drawAttitudeScreen();
            break;
        case SCREEN_COMPASS:
            drawCompassScreen();
            break;
        case SCREEN_ALTIMETER:
            drawAltimeterScreen();
            break;
        case SCREEN_AIRPLANE:
            drawAirplaneScreen();
            break;
        default:
            break;
    }
    g_tft->flush();
}

void pollTouchScreen()
{
    static bool touchWasDown = false;
    static uint16_t touchX = 0;
    static uint16_t touchY = 0;
    const bool touchDown = readTouch(touchX, touchY);
    if (touchDown && !touchWasDown) {
        g_currentScreen = static_cast<ScreenMode>((g_currentScreen + 1) % SCREEN_COUNT);
    }
    touchWasDown = touchDown;
}

}  // namespace

void setup()
{
    Serial.begin(115200);
    Wire.begin(aircraft::kI2cSdaPin, aircraft::kI2cSclPin);
    Wire.setClock(100000);

    if (!initializePmic()) {
        Serial.println("AXP2101 was not detected; display rails may stay off.");
    }
    resetTouchController();

    if (!g_tft->begin()) {
        Serial.println("Display initialization failed.");
        while (true) {
            delay(1000);
        }
    }
    g_display->setBrightness(200);
    g_tft->fillScreen(kBlack);

    pinMode(aircraft::kTouchInterruptPin, INPUT_PULLUP);
    g_imuReady = initializeImu();
    if (g_imuReady) {
        Serial.println("QMI8658 ready; level attitude requires +X acceleration near 1 g.");
    } else {
        Serial.println("QMI8658 not detected; attitude will not update.");
    }
    renderBootScreen();

    Serial.println("Aircraft Instruments booting...");
    Serial.println("Waveshare ESP32-S3 AMOLED ready.");
    delay(1500);
}

void loop()
{
    pollTouchScreen();
    updateImu();
    updateAltitudeFromPitch();

    renderScreen();
    delay(20);
}
