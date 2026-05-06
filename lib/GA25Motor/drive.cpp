/**
 * @file drive.cpp
 * @brief Triển khai Drive – Facade Motor + Encoder (ESP32Encoder) + PID
 */

#include "drive.h"

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
void Drive::motor_run(int speed) { _motor.motor_run(speed); }

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

    // output ∈ [-255, +255] → motor
    _motor.motor_run((int)output);
  }
}

// ─────────────────────────────────────────────
//  driveDistance() – chạy thẳng theo mm (blocking)
// ─────────────────────────────────────────────

void Drive::driveDistance(float mm, int speed) {
  if (mm == 0.0f || speed == 0)
    return;

  // ── 1. Tính số xung encoder tương ứng với mm ──────────────────────
  //   circumference  = PI × đường kính bánh (mm)
  //   số vòng cần    = mm / circumference
  //   số xung cần    = số_vòng × xung/vòng
  const float circumference = (float)PI * WHEEL_DIAMETER_MM;
  int32_t countsPerRev = _encoder.getCountsPerRev();
  long targetCounts = (long)(fabsf(mm) / circumference * (float)countsPerRev);

  if (targetCounts <= 0)
    return;

  // ── 2. Reset encoder & xác định chiều ─────────────────────────────
  encoder_reset(); // Reset cả encoder lẫn PID integral

  // mm > 0 → tiến (output dương), mm < 0 → lùi (output âm)
  int pwm = (mm > 0.0f) ? abs(speed) : -abs(speed);

  // ── 3. Tắt PID (chạy open-loop theo duty cố định) ─────────────────
  bool pidWasOn = _pidEnabled;
  enablePID(false);

  motor_run(pwm);

  // ── 4. Chờ đủ xung (polling encoder) ──────────────────────────────
  while (labs(encoder_get_count()) < targetCounts) {
    delay(1); // yield 1 ms, tránh watchdog trên ESP32
  }

  // ── 5. Dừng & khôi phục trạng thái PID ───────────────────────────
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
