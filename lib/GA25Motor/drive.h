#ifndef DRIVE_H
#define DRIVE_H

#include <Arduino.h>
#include "motor.h"
#include "encoder.h"
#include "pid.h"

// Chu kỳ update tốc độ/PID của Drive
#ifndef DRIVE_UPDATE_MS
#define DRIVE_UPDATE_MS 20
#endif

// Đường kính bánh xe GA25 của bạn: bán kính 32.5mm => đường kính 65mm
#ifndef WHEEL_DIAMETER_MM
#define WHEEL_DIAMETER_MM 65.0f
#endif

// =====================================================
// Class Drive: điều khiển 1 motor + 1 encoder + PID tốc độ
// =====================================================
class Drive {
public:
  Drive();

  void motor_init(uint8_t in1, uint8_t in2, uint8_t pwmPin,
                  uint8_t encA, uint8_t encB,
                  uint8_t ledcCh,
                  int32_t countsPerRev = 194,
                  uint32_t updateMs = DRIVE_UPDATE_MS);

  // Điều khiển motor thô
  void motor_set_pwm(int pwm);
  void motor_set_dir(int dir);
  void motor_stop();
  void motor_brake();
  void motor_run(int speed);

  // Chạy có dấu: speed > 0 tiến, speed < 0 lùi, speed = 0 dừng
  // Dùng hàm này để tránh lỗi speed âm bị ép về 0.
  void runSigned(int speed);

  // Encoder
  long encoder_get_count();
  void encoder_reset();

  // Tốc độ & PID
  float motor_get_speed();
  float motor_get_target_speed();
  void motor_set_target_speed(float target);
  void motor_update();

  // Chạy 1 motor theo khoảng cách mm, blocking
  void driveDistance(float mm, int speed);

  // PID config
  void setPIDGains(float kp, float ki, float kd);
  void enablePID(bool en);
  bool isPIDEnabled() const;
  void setIntegralLimit(float lim);

  // Truy cập module con
  Motor &getMotor();
  Encoder &getEncoder();
  PID &getPID();

  void printDebug();

private:
  Motor _motor;
  Encoder _encoder;
  PID _pid;

  bool _pidEnabled;
  float _targetRPM;
  uint32_t _updateMs;
  uint32_t _lastUpdate;
};

// =====================================================
// Class RobotDrive: wrapper 2 bánh trái/phải
// Gộp các hàm từ main cũ vào thư viện driver.
// Chỉ cần #include "drive.h" rồi tạo RobotDrive bot;
// =====================================================
class RobotDrive {
public:
  RobotDrive();

  // Begin mặc định theo chân trong main cũ của bạn:
  // Trái: AIN1=25, AIN2=33, PWMA=32, ENAL=36, ENBL=39, CH_A=0
  // Phải: BIN1=27, BIN2=26, PWMB=14, ENBR=35, ENAR=34, CH_B=1
  void begin();

  // Begin tự truyền chân nếu cần đổi mapping
  void begin(uint8_t lIn1, uint8_t lIn2, uint8_t lPwm,
             uint8_t lEncA, uint8_t lEncB, uint8_t lLedcCh,
             uint8_t rIn1, uint8_t rIn2, uint8_t rPwm,
             uint8_t rEncA, uint8_t rEncB, uint8_t rLedcCh,
             int32_t countsPerRev = 194,
             uint32_t updateMs = DRIVE_UPDATE_MS);

  void update();

  // Điều khiển 2 bánh trực tiếp, có hỗ trợ tốc độ âm.
  // Ví dụ: runRaw(130, -130) => trái tiến, phải lùi.
  void runRaw(int leftSpeed, int rightSpeed);
  void stop();
  void brake();

  void resetEncoders();
  long leftCount();
  long rightCount();

  // Chạy cứng theo khoảng cách, blocking.
  // Hệ số mặc định lấy từ main cũ: 1mm = 0.9388 xung.
  void runDistance(float mm, int speed = 100);

  // Quay tại chỗ theo góc, blocking.
  // angle > 0 quay phải, angle < 0 quay trái.
  // Hệ số mặc định lấy từ main cũ: 1 độ = 1.3333 xung.
  void turnAngle(float angle, int speed = 120);

  // Xoay tại chỗ không giới hạn encoder.
  // speed > 0 quay phải, speed < 0 quay trái.
  void pivot(int speed = 130);

  // Chỉnh hệ số thực nghiệm nếu test lại xe.
  void setDistanceCalib(float countsPerMm);
  void setTurnCalib(float countsPerDeg, int overshootOffset = 5);

  // Đảo chiều logic nếu bánh nào chạy ngược thực tế.
  // Giá trị nên là 1 hoặc -1.
  void setDirectionSign(int leftSign, int rightSign);

  Drive &left();
  Drive &right();

private:
  Drive _left;
  Drive _right;

  float _countsPerMm;
  float _countsPerDeg;
  int _turnOvershootOffset;

  int _leftSign;
  int _rightSign;

  static void signedRun(Drive &motor, int speed);
  static void stopOne(Drive &motor);
  static void brakeOne(Drive &motor);
};

#endif
