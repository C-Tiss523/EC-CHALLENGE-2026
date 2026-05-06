/**
 * @file motor.h
 * @brief Điều khiển 1 kênh motor DC qua TB6612FNG + LEDC (ESP32)
 *
 * Module này chỉ xử lý phần điện cơ: chiều quay + duty cycle PWM.
 * Không phụ thuộc encoder hay PID.
 *
 * @author  EC Challenge 2026
 * @version 2.0.0
 * @date    2026-04-19
 */

#pragma once
#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

#define MOTOR_PWM_FREQ       20000   ///< Tần số PWM (Hz)
#define MOTOR_PWM_RESOLUTION 8       ///< Độ phân giải PWM (bit)
#define MOTOR_PWM_MAX        255     ///< Giá trị PWM tối đa

/**
 * @brief Điều khiển 1 kênh motor qua TB6612FNG
 *
 * Bảng chiều quay TB6612FNG:
 * | IN1 | IN2 | PWM | Kết quả      |
 * |-----|-----|-----|--------------|
 * |  H  |  L  |  ~  | Tiến (CW)   |
 * |  L  |  H  |  ~  | Lùi  (CCW)  |
 * |  H  |  H  |  ~  | Short brake |
 * |  L  |  L  |  ~  | Coast/Free  |
 *
 * @code
 *   Motor m;
 *   m.init(25, 26, 27, 0);  // IN1, IN2, PWM pin, LEDC channel
 *   m.run(150);             // tiến, duty 150
 *   m.run(-200);            // lùi,  duty 200
 *   m.stop();               // coast
 *   m.brake();              // short brake
 * @endcode
 */
class Motor {
public:
    Motor();

    /**
     * @brief Khởi tạo chân GPIO và LEDC
     * @param in1     Chân AIN1/BIN1
     * @param in2     Chân AIN2/BIN2
     * @param pwmPin  Chân PWMA/PWMB
     * @param ledcCh  Kênh LEDC (0–15), mỗi motor dùng 1 kênh riêng
     */
    void init(uint8_t in1, uint8_t in2, uint8_t pwmPin, uint8_t ledcCh = 0);

    void motor_set_pwm(int pwm);    ///< Đặt duty PWM 0–255
    void motor_set_dir(int dir);    ///< 1 = tiến, -1 = lùi, 0 = coast
    void motor_stop();              ///< Coast (thả tự do)
    void brake();                   ///< Short brake (IN1=H, IN2=H)
    void motor_run(int speed);      ///< speed âm/dương -255…+255

    uint8_t getPWM()  const;        ///< Lấy duty hiện tại
    int     getDir()  const;        ///< Lấy chiều hiện tại: 1, -1, 0
    void    setInverted(bool inv);  ///< Đảo logic chiều quay

private:
    uint8_t _in1, _in2, _pwmPin, _ledcCh;
    bool    _inverted;
    uint8_t _currentPWM;
    int     _currentDir;

    void _applyDir(int dir);
    void _applyPWM(uint8_t duty);
};

#endif // MOTOR_H
