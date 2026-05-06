/**
 * @file encoder.h
 * @brief Wrapper đọc encoder quadrature dùng ESP32Encoder (Kevin Harrington)
 *
 * Thư viện ESP32Encoder dùng PCNT (Pulse Counter) hardware của ESP32,
 * chính xác hơn ISR phần mềm và không miss pulse ở tốc độ cao.
 *
 * GitHub: https://github.com/madhephaestus/ESP32Encoder
 * PlatformIO: madhephaestus/ESP32Encoder
 *
 * Thông số GA25-370 mặc định:
 *  - PPR (trước hộp số): 11 xung/vòng
 *  - Tỉ số hộp số: 34
 *  - Full quadrature (x4): 11 × 34 × 4 = 1496 xung/vòng trục ra
 *
 * @author  EC Challenge 2026
 * @version 2.0.0
 * @date    2026-04-19
 */

#pragma once
#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include <ESP32Encoder.h>

#define ENC_PPR             11                          ///< Xung/vòng trục motor
#define ENC_GEAR_RATIO      4.4                          ///< Tỉ số hộp số GA25-370
#define ENC_COUNTS_PER_REV  (ENC_PPR * ENC_GEAR_RATIO * 4)  ///< = 194   ///< Xung/vòng trục ra (full quadrature)

/**
 * @brief Đọc encoder quadrature GA25-370 qua ESP32Encoder (PCNT hardware)
 *
 * @code
 *   Encoder enc;
 *   enc.encoder_init(34, 35);       // Phase A, Phase B
 *
 *   // Trong loop():
 *   enc.update();
 *   long  cnt = enc.encoder_get_count();  // Tổng xung
 *   float rpm = enc.getRPM();             // RPM trục ra (filtered)
 * @endcode
 */
class Encoder {
public:
    Encoder();

    /**
     * @brief Khởi tạo encoder
     * @param pinA          Chân Phase A (ngắt, ví dụ GPIO 34)
     * @param pinB          Chân Phase B (ví dụ GPIO 35)
     * @param countsPerRev  Số xung/vòng trục ra (mặc định 1496)
     */
    void encoder_init(uint8_t pinA, uint8_t pinB,
                      int32_t countsPerRev = ENC_COUNTS_PER_REV);

    /**
     * @brief Lấy tổng xung tích lũy (có dấu)
     * @return Số xung (long)
     */
    long encoder_get_count();

    /**
     * @brief Reset bộ đếm về 0
     */
    void encoder_reset();

    /**
     * @brief Lấy tốc độ xung/giây (raw, cập nhật sau mỗi update())
     */
    float getSpeedPPS() const;

    /**
     * @brief Lấy số xung/vòng đã cấu hình
     */
    int32_t getCountsPerRev() const;

    /**
     * @brief Lấy RPM trục ra (đã lọc EMA)
     * @return RPM (âm = quay nghịch)
     */
    float getRPM() const;

    /**
     * @brief Lấy góc tích lũy (độ)
     */
    float getAngleDeg();

    /**
     * @brief Lấy số vòng quay tích lũy
     */
    float getRevolutions();

    /**
     * @brief Đặt hệ số lọc EMA
     * @param alpha  0.0 (rất mượt) – 1.0 (không lọc), mặc định 0.3
     */
    void setFilterAlpha(float alpha);

    /**
     * @brief Cập nhật tốc độ – gọi liên tục trong loop()
     * @param intervalMs  Chu kỳ tính lại (ms). Bỏ qua nếu chưa đến chu kỳ.
     */
    void update(uint32_t intervalMs = 20);

private:
    ESP32Encoder _enc;          ///< ESP32Encoder (Kevin Harrington)
    int32_t      _countsPerRev;

    long         _lastCount;
    float        _speedPPS;     ///< Xung/giây (raw)
    float        _rpmFiltered;  ///< RPM sau lọc EMA
    float        _alpha;        ///< Hệ số lọc EMA
    uint32_t     _lastTime;     ///< Thời điểm update lần trước
};

#endif // ENCODER_H
