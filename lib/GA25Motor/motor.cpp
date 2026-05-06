/**
 * @file motor.cpp
 * @brief Triển khai Motor – TB6612FNG + LEDC (ESP32)
 */

#include "motor.h"

Motor::Motor()
    : _in1(0), _in2(0), _pwmPin(0), _ledcCh(0),
      _inverted(false), _currentPWM(0), _currentDir(0)
{}

void Motor::init(uint8_t in1, uint8_t in2, uint8_t pwmPin, uint8_t ledcCh)
{
    _in1    = in1;
    _in2    = in2;
    _pwmPin = pwmPin;
    _ledcCh = ledcCh;

    // Cấu hình chân điều hướng
    pinMode(_in1, OUTPUT);
    pinMode(_in2, OUTPUT);
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, LOW);

    // Cấu hình LEDC PWM phần cứng ESP32
    ledcSetup(_ledcCh, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
    ledcAttachPin(_pwmPin, _ledcCh);
    ledcWrite(_ledcCh, 0);

    _currentPWM = 0;
    _currentDir = 0;
}

// ─── Helper nội bộ ────────────────────────────────

void Motor::_applyDir(int dir)
{
    if (_inverted) dir = -dir;
    _currentDir = dir;

    if (dir > 0) {
        digitalWrite(_in1, HIGH);
        digitalWrite(_in2, LOW);
    } else if (dir < 0) {
        digitalWrite(_in1, LOW);
        digitalWrite(_in2, HIGH);
    } else {
        // COAST
        digitalWrite(_in1, LOW);
        digitalWrite(_in2, LOW);
    }
}

void Motor::_applyPWM(uint8_t duty)
{
    _currentPWM = duty;
    ledcWrite(_ledcCh, duty);
}

// ─── API công khai ────────────────────────────────

void Motor::motor_set_pwm(int pwm)
{
    _applyPWM((uint8_t)constrain(pwm, 0, MOTOR_PWM_MAX));
}

void Motor::motor_set_dir(int dir)
{
    _applyDir(dir);
}

void Motor::motor_stop()
{
    _applyDir(0);
    _applyPWM(0);
}

void Motor::brake()
{
    // SHORT BRAKE: IN1=H, IN2=H
    _currentDir = 0;
    digitalWrite(_in1, HIGH);
    digitalWrite(_in2, HIGH);
    _applyPWM(MOTOR_PWM_MAX);
}

void Motor::motor_run(int speed)
{
    if (speed > 0) {
        _applyDir(1);
        _applyPWM((uint8_t)constrain(speed, 0, MOTOR_PWM_MAX));
    } else if (speed < 0) {
        _applyDir(-1);
        _applyPWM((uint8_t)constrain(-speed, 0, MOTOR_PWM_MAX));
    } else {
        motor_stop();
    }
}

void Motor::setInverted(bool inv) { _inverted = inv; }
uint8_t Motor::getPWM()  const    { return _currentPWM; }
int     Motor::getDir()  const    { return _currentDir; }
