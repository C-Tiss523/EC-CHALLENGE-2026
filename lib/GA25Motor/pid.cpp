/**
 * @file pid.cpp
 * @brief Triển khai bộ điều khiển PID thuần túy
 */

#include "pid.h"

PID::PID(float kp, float ki, float kd)
    : _kp(kp), _ki(ki), _kd(kd),
      _target(0.0f),
      _integral(0.0f),
      _prevError(0.0f),
      _outputLimit(255.0f),
      _integralLimit(200.0f)
{}

// ─── Cấu hình ──────────────────────────────────────

void PID::setGains(float kp, float ki, float kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
    reset();
}

void PID::motor_set_target_speed(float target)
{
    if (fabsf(target - _target) > 0.001f) {
        _integral  = 0.0f;
        _prevError = 0.0f;
    }
    _target = target;
}

float PID::motor_get_target_speed() const { return _target; }

void PID::setOutputLimit(float limit)   { _outputLimit   = fabsf(limit); }
void PID::setIntegralLimit(float limit) { _integralLimit = fabsf(limit); }

// ─── compute() ─────────────────────────────────────

float PID::compute(float measured, float dt)
{
    if (dt <= 0.0f) return 0.0f;

    float error = _target - measured;

    // Tích phân + anti-windup clamp
    _integral += error * dt;
    _integral  = constrain(_integral, -_integralLimit, _integralLimit);

    // Vi phân
    float derivative = (error - _prevError) / dt;
    _prevError = error;

    // Đầu ra PID
    float output = _kp * error
                 + _ki * _integral
                 + _kd * derivative;

    return constrain(output, -_outputLimit, _outputLimit);
}

void PID::reset()
{
    _integral  = 0.0f;
    _prevError = 0.0f;
}

// ─── Getters ───────────────────────────────────────

float PID::getKp()        const { return _kp;        }
float PID::getKi()        const { return _ki;        }
float PID::getKd()        const { return _kd;        }
float PID::getIntegral()  const { return _integral;  }
float PID::getLastError() const { return _prevError; }
