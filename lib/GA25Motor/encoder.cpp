/**
 * @file encoder.cpp
 * @brief Triển khai Encoder – dùng ESP32Encoder của Kevin Harrington (PCNT hardware)
 *
 * API ESP32Encoder chính:
 *   ESP32Encoder::useInternalWeakPullResistors = puType::up;
 *   _enc.attachFullQuad(pinA, pinB);  // quadrature x4
 *   _enc.setCount(0);                 // reset
 *   (int64_t)_enc.getCount();         // đọc xung
 */

#include "encoder.h"

Encoder::Encoder()
    : _countsPerRev(ENC_COUNTS_PER_REV),
      _lastCount(0),
      _speedPPS(0.0f),
      _rpmFiltered(0.0f),
      _alpha(0.3f),
      _lastTime(0)
{}

// ─────────────────────────────────────────────
//  encoder_init()
// ─────────────────────────────────────────────

void Encoder::encoder_init(uint8_t pinA, uint8_t pinB, int32_t countsPerRev)
{
    _countsPerRev = countsPerRev;

    // Bật pull-up nội (GPIO 34/35 không có → cần pull-up ngoài 10kΩ)
    ESP32Encoder::useInternalWeakPullResistors = puType::up;

    // Full quadrature (x4): đếm cả 4 cạnh A và B
    _enc.attachFullQuad(pinA, pinB);
    _enc.setCount(0);

    _lastCount   = 0;
    _lastTime    = millis();
    _speedPPS    = 0.0f;
    _rpmFiltered = 0.0f;
}

// ─────────────────────────────────────────────
//  update() – tính tốc độ theo chu kỳ
// ─────────────────────────────────────────────

void Encoder::update(uint32_t intervalMs)
{
    uint32_t now = millis();
    uint32_t dt  = now - _lastTime;

    if (dt < intervalMs) return;   // Chưa đến chu kỳ, bỏ qua

    // Đọc xung từ PCNT hardware
    long currentCount = (long)_enc.getCount();
    long delta        = currentCount - _lastCount;

    // Tốc độ: xung/giây
    float dtSec = (float)dt * 0.001f;
    _speedPPS = (dtSec > 0.0f) ? ((float)delta / dtSec) : 0.0f;

    // RPM = (xung/giây ÷ xung/vòng) × 60 giây/phút
    float rawRPM = (_countsPerRev > 0)
                   ? (_speedPPS / (float)_countsPerRev * 60.0f)
                   : 0.0f;

    // Lọc EMA (Exponential Moving Average): mượt hơn raw RPM
    _rpmFiltered = _alpha * rawRPM + (1.0f - _alpha) * _rpmFiltered;

    _lastCount = currentCount;
    _lastTime  = now;
}

// ─────────────────────────────────────────────
//  API công khai
// ─────────────────────────────────────────────

long Encoder::encoder_get_count()
{
    return (long)_enc.getCount();
}

void Encoder::encoder_reset()
{
    _enc.setCount(0);
    _lastCount   = 0;
    _speedPPS    = 0.0f;
    _rpmFiltered = 0.0f;
}

float Encoder::getSpeedPPS()    const { return _speedPPS;    }
float Encoder::getRPM()         const { return _rpmFiltered; }
int32_t Encoder::getCountsPerRev() const { return _countsPerRev; }

float Encoder::getAngleDeg()
{
    if (_countsPerRev == 0) return 0.0f;
    return ((float)_enc.getCount() / (float)_countsPerRev) * 360.0f;
}

float Encoder::getRevolutions()
{
    if (_countsPerRev == 0) return 0.0f;
    return (float)_enc.getCount() / (float)_countsPerRev;
}

void Encoder::setFilterAlpha(float alpha)
{
    _alpha = constrain(alpha, 0.0f, 1.0f);
}
