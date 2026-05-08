/**
 * @file drive.cpp
 * @brief Triển khai Drive – Facade Motor + Encoder (ESP32Encoder) + PID
 *        và RobotDrive – wrapper 2 bánh trái/phải cho xe.
 */

#include "drive.h"

// =====================================================
// Drive – 1 motor + 1 encoder + PID
// =====================================================

Drive::Drive()
    : _pid(2.0f, 1.0f, 0.05f), _pidEnabled(false), _targetRPM(0.0f),
      _updateMs(DRIVE_UPDATE_MS), _lastUpdate(0) {}

// ─────────────────────────────────────────────
//  motor_init()
// ─────────────────────────────────────────────

void Drive::motor_init(uint8_t in1, uint8_t in2, uint8_t pwmPin, uint8_t encA,
                       uint8_t encB, uint8_t ledcCh, int32_t countsPerRev,
                       uint32_t updateMs) {
  _updateMs = updateMs;

  _motor.init(in1, in2, pwmPin, ledcCh);
  _encoder.encoder_init(encA, encB, countsPerRev);

  _pid.setOutputLimit(255.0f);
  _pid.setIntegralLimit(200.0f);

  _lastUpdate = millis();
}

// ─────────────────────────────────────────────
//  Điều khiển motor thô
// ─────────────────────────────────────────────

void Drive::motor_set_pwm(int pwm) { _motor.motor_set_pwm(pwm); }
void Drive::motor_set_dir(int dir) { _motor.motor_set_dir(dir); }
void Drive::motor_stop() { _motor.motor_stop(); }
void Drive::motor_brake() { _motor.brake(); }
void Drive::motor_run(int speed) { _motor.motor_run(speed); }

// Chạy có dấu: dương = tiến, âm = lùi, 0 = dừng.
// Hàm này cố tình KHÔNG dùng motor_run(speed âm), để tránh lỗi speed âm bị constrain về 0.
void Drive::runSigned(int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    _motor.motor_set_dir(1);
    _motor.motor_set_pwm(speed);
  } else if (speed < 0) {
    _motor.motor_set_dir(-1);
    _motor.motor_set_pwm(-speed);
  } else {
    _motor.motor_stop();
  }
}

// ─────────────────────────────────────────────
//  Encoder
// ─────────────────────────────────────────────

long Drive::encoder_get_count() { return _encoder.encoder_get_count(); }

void Drive::encoder_reset() {
  _encoder.encoder_reset();
  _pid.reset(); // Reset PID luôn tránh spike sau reset
}

// ─────────────────────────────────────────────
//  Tốc độ & PID
// ─────────────────────────────────────────────

float Drive::motor_get_speed() { return _encoder.getRPM(); }
float Drive::motor_get_target_speed() { return _targetRPM; }

void Drive::motor_set_target_speed(float target) {
  _targetRPM = target;
  _pid.motor_set_target_speed(target);
}

// ─────────────────────────────────────────────
//  motor_update() – vòng lặp chính
// ─────────────────────────────────────────────

void Drive::motor_update() {
  uint32_t now = millis();
  uint32_t dt = now - _lastUpdate;

  if (dt < _updateMs)
    return; // Chưa đến chu kỳ
  _lastUpdate = now;

  // 1. Cập nhật encoder (tính RPM)
  _encoder.update(_updateMs);

  // 2. Chạy PID nếu bật
  if (_pidEnabled) {
    float dtSec = (float)dt * 0.001f;
    float rpm = _encoder.getRPM();
    float output = _pid.compute(rpm, dtSec);

    // output ∈ [-255, +255] → motor, dùng runSigned để xử lý chiều âm chắc chắn
    runSigned((int)output);
  }
}

// ─────────────────────────────────────────────
//  driveDistance() – chạy 1 motor theo mm (blocking)
// ─────────────────────────────────────────────

void Drive::driveDistance(float mm, int speed) {
  if (mm == 0.0f || speed == 0)
    return;

  // circumference = PI × đường kính bánh (mm)
  const float circumference = (float)PI * WHEEL_DIAMETER_MM;
  int32_t countsPerRev = _encoder.getCountsPerRev();
  long targetCounts = (long)(fabsf(mm) / circumference * (float)countsPerRev);

  if (targetCounts <= 0)
    return;

  encoder_reset(); // Reset cả encoder lẫn PID integral

  int pwm = (mm > 0.0f) ? abs(speed) : -abs(speed);

  bool pidWasOn = _pidEnabled;
  enablePID(false);

  runSigned(pwm);

  while (labs(encoder_get_count()) < targetCounts) {
    delay(1); // yield 1 ms, tránh watchdog trên ESP32
  }

  motor_stop();
  if (pidWasOn)
    enablePID(true);
}

// ─────────────────────────────────────────────
//  Cấu hình PID
// ─────────────────────────────────────────────

void Drive::setPIDGains(float kp, float ki, float kd) {
  _pid.setGains(kp, ki, kd);
}

void Drive::enablePID(bool en) {
  _pidEnabled = en;
  if (!en)
    _pid.reset();
}

bool Drive::isPIDEnabled() const { return _pidEnabled; }

void Drive::setIntegralLimit(float lim) { _pid.setIntegralLimit(lim); }

// ─────────────────────────────────────────────
//  Truy cập module con
// ─────────────────────────────────────────────

Motor &Drive::getMotor() { return _motor; }
Encoder &Drive::getEncoder() { return _encoder; }
PID &Drive::getPID() { return _pid; }

// ─────────────────────────────────────────────
//  printDebug()
// ─────────────────────────────────────────────

void Drive::printDebug() {
  Serial.print(F("[Drive] Enc="));
  Serial.print(encoder_get_count());
  Serial.print(F("  RPM="));
  Serial.print(_encoder.getRPM(), 1);
  Serial.print(F("  Target="));
  Serial.print(_targetRPM, 1);
  Serial.print(F("  PWM="));
  Serial.print(_motor.getPWM());
  Serial.print(F("  Dir="));
  Serial.print(_motor.getDir());
  Serial.print(F("  PID="));
  Serial.println(_pidEnabled ? F("ON") : F("OFF"));
}

// =====================================================
// RobotDrive – wrapper 2 bánh trái/phải
// =====================================================

namespace {
// Mapping mặc định theo main cũ của bạn
const uint8_t DEF_AIN1 = 25;
const uint8_t DEF_AIN2 = 33;
const uint8_t DEF_PWMA = 32;
const uint8_t DEF_ENAL = 36;
const uint8_t DEF_ENBL = 39;
const uint8_t DEF_CHA  = 0;

const uint8_t DEF_BIN1 = 27;
const uint8_t DEF_BIN2 = 26;
const uint8_t DEF_PWMB = 14;
const uint8_t DEF_ENAR = 34;
const uint8_t DEF_ENBR = 35;
const uint8_t DEF_CHB  = 1;
}

RobotDrive::RobotDrive()
    : _countsPerMm(0.9388f), _countsPerDeg(1.3333f),
      _turnOvershootOffset(5), _leftSign(1), _rightSign(1) {}

void RobotDrive::begin() {
  // Giữ đúng logic main cũ:
  // motorL.motor_init(AIN1, AIN2, PWMA, ENAL, ENBL, CH_A);
  // motorR.motor_init(BIN1, BIN2, PWMB, ENBR, ENAR, CH_B);
  begin(DEF_AIN1, DEF_AIN2, DEF_PWMA, DEF_ENAL, DEF_ENBL, DEF_CHA,
        DEF_BIN1, DEF_BIN2, DEF_PWMB, DEF_ENBR, DEF_ENAR, DEF_CHB,
        194, DRIVE_UPDATE_MS);
}

void RobotDrive::begin(uint8_t lIn1, uint8_t lIn2, uint8_t lPwm,
                       uint8_t lEncA, uint8_t lEncB, uint8_t lLedcCh,
                       uint8_t rIn1, uint8_t rIn2, uint8_t rPwm,
                       uint8_t rEncA, uint8_t rEncB, uint8_t rLedcCh,
                       int32_t countsPerRev, uint32_t updateMs) {
  _left.motor_init(lIn1, lIn2, lPwm, lEncA, lEncB, lLedcCh, countsPerRev, updateMs);
  _right.motor_init(rIn1, rIn2, rPwm, rEncA, rEncB, rLedcCh, countsPerRev, updateMs);
  resetEncoders();
}

void RobotDrive::update() {
  _left.motor_update();
  _right.motor_update();
}

void RobotDrive::signedRun(Drive &motor, int speed) {
  motor.runSigned(speed);
}

void RobotDrive::stopOne(Drive &motor) {
  motor.motor_stop();
}

void RobotDrive::brakeOne(Drive &motor) {
  motor.motor_brake();
}

void RobotDrive::runRaw(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  signedRun(_left, leftSpeed * _leftSign);
  signedRun(_right, rightSpeed * _rightSign);
}

void RobotDrive::stop() {
  stopOne(_left);
  stopOne(_right);
}

void RobotDrive::brake() {
  brakeOne(_left);
  brakeOne(_right);
}

void RobotDrive::resetEncoders() {
  _left.encoder_reset();
  _right.encoder_reset();
}

long RobotDrive::leftCount() { return _left.encoder_get_count(); }
long RobotDrive::rightCount() { return _right.encoder_get_count(); }

void RobotDrive::runDistance(float mm, int speed) {
  long targetCounts = (long)(fabsf(mm) * _countsPerMm);
  if (targetCounts <= 0 || speed == 0)
    return;

  resetEncoders();

  int pwm = (mm > 0.0f) ? abs(speed) : -abs(speed);
  runRaw(pwm, pwm);

  bool lDone = false;
  bool rDone = false;

  while (!lDone || !rDone) {
    if (!lDone && labs(_left.encoder_get_count()) >= targetCounts) {
      stopOne(_left);
      lDone = true;
    }

    if (!rDone && labs(_right.encoder_get_count()) >= targetCounts) {
      stopOne(_right);
      rDone = true;
    }

    delay(1); // Tránh watchdog reset
  }
}

void RobotDrive::turnAngle(float angle, int speed) {
  long targetCounts = (long)(fabsf(angle) * _countsPerDeg) - _turnOvershootOffset;
  if (targetCounts <= 0 || speed == 0)
    return;

  resetEncoders();

  // angle > 0: quay phải -> trái tiến, phải lùi
  // angle < 0: quay trái -> trái lùi, phải tiến
  int pwmL = (angle > 0.0f) ? abs(speed) : -abs(speed);
  int pwmR = (angle > 0.0f) ? -abs(speed) : abs(speed);

  runRaw(pwmL, pwmR);

  bool lDone = false;
  bool rDone = false;

  while (!lDone || !rDone) {
    if (!lDone && labs(_left.encoder_get_count()) >= targetCounts) {
      brakeOne(_left);
      lDone = true;
    }

    if (!rDone && labs(_right.encoder_get_count()) >= targetCounts) {
      brakeOne(_right);
      rDone = true;
    }

    delay(1); // Tránh watchdog reset
  }
}

void RobotDrive::pivot(int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    runRaw(abs(speed), -abs(speed)); // quay phải
  } else if (speed < 0) {
    runRaw(-abs(speed), abs(speed)); // quay trái
  } else {
    stop();
  }
}

void RobotDrive::setDistanceCalib(float countsPerMm) {
  if (countsPerMm > 0.0f)
    _countsPerMm = countsPerMm;
}

void RobotDrive::setTurnCalib(float countsPerDeg, int overshootOffset) {
  if (countsPerDeg > 0.0f)
    _countsPerDeg = countsPerDeg;
  _turnOvershootOffset = overshootOffset;
}

void RobotDrive::setDirectionSign(int leftSign, int rightSign) {
  _leftSign = (leftSign >= 0) ? 1 : -1;
  _rightSign = (rightSign >= 0) ? 1 : -1;
}

Drive &RobotDrive::left() { return _left; }
Drive &RobotDrive::right() { return _right; }
