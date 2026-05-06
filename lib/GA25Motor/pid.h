/**
 * @file pid.h
 * @brief Bộ điều khiển PID độc lập – tái sử dụng cho mọi ứng dụng vòng kín
 *
 * Đặc điểm:
 *  - Anti-windup: clamp tích phân
 *  - Output clamp: giới hạn đầu ra
 *  - Auto-reset integral khi đổi target
 *
 * @author  EC Challenge 2026
 * @version 2.0.0
 * @date    2026-04-19
 */

#pragma once
#ifndef PID_H
#define PID_H

#include <Arduino.h>

/**
 * @brief Bộ điều khiển PID vòng kín
 *
 * @code
 *   PID pid(2.0f, 1.0f, 0.05f);
 *   pid.motor_set_target_speed(100.0f);   // RPM mục tiêu
 *   pid.setOutputLimit(255.0f);
 *
 *   // Gọi đều đặn trong vòng lặp:
 *   float dt  = 0.020f;                   // 20ms → giây
 *   float out = pid.compute(measuredRPM, dt);
 * @endcode
 */
class PID {
public:
    /**
     * @param kp  Hệ số Proportional
     * @param ki  Hệ số Integral
     * @param kd  Hệ số Derivative
     */
    PID(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f);

    // ── Cấu hình ──────────────────────────────

    /** Đặt hệ số PID (tự reset trạng thái) */
    void setGains(float kp, float ki, float kd);

    /**
     * @brief Đặt giá trị mục tiêu (setpoint)
     * Reset integral/prevError nếu target thay đổi đáng kể.
     */
    void motor_set_target_speed(float target);

    /** Lấy giá trị mục tiêu hiện tại */
    float motor_get_target_speed() const;

    /** Giới hạn đầu ra ±limit */
    void setOutputLimit(float limit);

    /** Giới hạn tích phân chống wind-up ±limit */
    void setIntegralLimit(float limit);

    // ── Tính toán ─────────────────────────────

    /**
     * @brief Tính đầu ra PID một bước
     * @param measured  Giá trị đo được (RPM hiện tại)
     * @param dt        Thời gian từ lần gọi trước (giây, > 0)
     * @return          Đầu ra điều khiển (đã clamp)
     */
    float compute(float measured, float dt);

    /** Reset integral và prevError */
    void reset();

    // ── Getters ───────────────────────────────
    float getKp()        const;
    float getKi()        const;
    float getKd()        const;
    float getIntegral()  const;
    float getLastError() const;

private:
    float _kp, _ki, _kd;
    float _target;
    float _integral;
    float _prevError;
    float _outputLimit;
    float _integralLimit;
};

#endif // PID_H
