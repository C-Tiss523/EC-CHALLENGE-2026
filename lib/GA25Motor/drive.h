/**
 * @file drive.h
 * @brief Facade tích hợp Motor + Encoder + PID cho GA25-370 + TB6612FNG
 *
 * Class chính người dùng tương tác. Ghép 3 module:
 *  - Motor  : điều khiển TB6612FNG + LEDC
 *  - Encoder: đọc xung qua ESP32Encoder (Kevin Harrington, PCNT hardware)
 *  - PID    : điều khiển tốc độ vòng kín
 *
 * Sơ đồ kết nối ví dụ (ESP32 + TB6612FNG + GA25-370):
 * ┌────────────────────────────────────────────┐
 * │  TB6612FNG    ESP32                         │
 * │  STBY     →  3.3V                          │
 * │  AIN1     →  GPIO 25                       │
 * │  AIN2     →  GPIO 26                       │
 * │  PWMA     →  GPIO 27                       │
 * │  Encoder A → GPIO 34                       │
 * │  Encoder B → GPIO 35                       │
 * └────────────────────────────────────────────┘
 *
 * @code
 *   Drive motor;
 *
 *   void setup() {
 *       motor.motor_init(25, 26, 27, 34, 35);
 *       motor.setPIDGains(2.0f, 1.0f, 0.05f);
 *       motor.enablePID(true);
 *       motor.motor_set_target_speed(100.0f);  // 100 RPM
 *   }
 *
 *   void loop() {
 *       motor.motor_update();   // Gọi liên tục – không blocking
 *   }
 * @endcode
 *
 * @author  EC Challenge 2026
 * @version 2.0.0
 * @date    2026-04-19
 */

#pragma once
#ifndef DRIVE_H
#define DRIVE_H

#include <Arduino.h>
#include "motor.h"
#include "encoder.h"
#include "pid.h"

#define DRIVE_UPDATE_MS      20      ///< Chu kỳ update mặc định (ms)

/**
 * @brief Đường kính bánh xe (mm) – chỉnh lại theo robot thực tế.
 *
 * Công thức tính quãng đường:
 *   circumference = PI × WHEEL_DIAMETER_MM
 *   target_counts = (mm / circumference) × countsPerRev
 */
#ifndef WHEEL_DIAMETER_MM
#define WHEEL_DIAMETER_MM    65.0f  ///< ← SỬA GIÁ TRỊ NÀY theo đường kính bánh (mm)
#endif

/**
 * @brief Facade điều khiển GA25-370 – tích hợp Motor + Encoder + PID
 */
class Drive {
public:
    Drive();

    // ── Khởi tạo ──────────────────────────────

    /**
     * @brief Khởi tạo toàn bộ hệ thống
     * @param in1           Chân AIN1/BIN1 (TB6612FNG)
     * @param in2           Chân AIN2/BIN2 (TB6612FNG)
     * @param pwmPin        Chân PWMA/PWMB (TB6612FNG)
     * @param encA          Chân encoder Phase A
     * @param encB          Chân encoder Phase B
     * @param ledcCh        Kênh LEDC ESP32 (0–15, mặc định 0)
     * @param countsPerRev  Số xung/vòng trục ra (mặc định 1496)
     * @param updateMs      Chu kỳ update RPM & PID (ms, mặc định 20)
     */
    void motor_init(uint8_t in1, uint8_t in2, uint8_t pwmPin,
                    uint8_t encA, uint8_t encB,
                    uint8_t  ledcCh       = 0,
                    int32_t  countsPerRev = ENC_COUNTS_PER_REV,
                    uint32_t updateMs     = DRIVE_UPDATE_MS);

    // ── Điều khiển motor thô ──────────────────

    void motor_set_pwm(int pwm);    ///< Đặt duty PWM 0–255
    void motor_set_dir(int dir);    ///< 1 = tiến, -1 = lùi
    void motor_stop();              ///< Dừng (coast)
    void motor_run(int speed);      ///< speed âm/dương -255…+255

    // ── Encoder ───────────────────────────────

    long encoder_get_count();       ///< Tổng xung tích lũy (có dấu)
    void encoder_reset();           ///< Reset xung về 0

    // ── Tốc độ ────────────────────────────────

    float motor_get_speed();                    ///< RPM hiện tại (filtered EMA)
    void  motor_set_target_speed(float target); ///< Đặt RPM mục tiêu cho PID
    float motor_get_target_speed();             ///< Lấy RPM mục tiêu

    // ── Vòng lặp chính ────────────────────────

    /**
     * @brief Cập nhật encoder + PID – gọi liên tục trong loop()
     * Không blocking; tự kiểm tra chu kỳ bên trong.
     */
    void motor_update();

    // ── Cấu hình PID ─────────────────────────

    void setPIDGains(float kp, float ki, float kd); ///< Đặt hệ số PID
    void enablePID(bool en);                         ///< Bật/tắt PID
    bool isPIDEnabled() const;                       ///< Kiểm tra PID
    void setIntegralLimit(float lim);                ///< Anti-windup

    // ── Truy cập module con ───────────────────

    Motor&   getMotor();    ///< Truy cập trực tiếp Motor
    Encoder& getEncoder();  ///< Truy cập trực tiếp Encoder
    PID&     getPID();      ///< Truy cập trực tiếp PID

    // ── Điều khiển theo quãng đường ──────────

    /**
     * @brief Chạy thẳng một quãng đường cho trước rồi dừng (blocking).
     *
     * Hàm sẽ BLOCK cho đến khi bánh xe đi đủ @p mm.
     * Dùng encoder để đo quãng đường thực tế → không phụ thuộc thời gian.
     *
     * @param mm     Quãng đường cần đi (mm).
     *               Dương = tiến, Âm = lùi.
     * @param speed  Duty PWM 0–255 (luôn dương, chiều do dấu mm quyết định).
     *               Mặc định 150.
     *
     * @note WHEEL_DIAMETER_MM phải được đặt đúng trước khi gọi hàm.
     *       Có thể override bằng `#define WHEEL_DIAMETER_MM <giá_trị>` trước
     *       khi #include "drive.h".
     */
    void driveDistance(float mm, int speed = 150);

    // ── Tiện ích ─────────────────────────────

    void printDebug(); ///< In debug ra Serial

private:
    Motor   _motor;
    Encoder _encoder;
    PID     _pid;

    bool     _pidEnabled;
    float    _targetRPM;
    uint32_t _updateMs;
    uint32_t _lastUpdate;
};

#endif // DRIVE_H
