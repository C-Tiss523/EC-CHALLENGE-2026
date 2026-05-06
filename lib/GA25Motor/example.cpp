// /**
//  * @file main.cpp
//  * @brief Test Encoder – Chạy motor open-loop (PWM cố định), in RPM + xung.
//  *
//  * PID tắt hoàn toàn. Dùng để kiểm tra encoder đọc đúng không.
//  *
//  * Lệnh Serial (baud 115200):
//  *   <số>   → Đặt PWM -255…+255  (vd: 180 = tiến, -180 = lùi)
//  *   0      → Dừng motor
//  *   r      → Reset bộ đếm encoder về 0
//  *
//  * Sơ đồ nối (chỉnh lại theo phần cứng):
//  *   TB6612FNG   ESP32
//  *   AIN1    →   GPIO 25
//  *   AIN2    →   GPIO 26
//  *   PWMA    →   GPIO 27
//  *   Encoder A → GPIO 34
//  *   Encoder B → GPIO 35
//  *   STBY    →   3.3 V
//  */

#include "drive.h"
#include <Arduino.h>

// ── Cấu hình chân (chỉnh theo phần cứng) ─────────────────────────────────────
#define PIN_IN1 25
#define PIN_IN2 26
#define PIN_PWM 27
#define PIN_ENC_A 34
#define PIN_ENC_B 35
#define LEDC_CH 0

// ── Chu kỳ in (ms) ───────────────────────────────────────────────────────────
#define PRINT_INTERVAL_MS 200

// ─────────────────────────────────────────────────────────────────────────────

Drive motor;

uint32_t lastPrint = 0;
int currentPWM = 0;

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println(F("========================================"));
  Serial.println(F("  GA25-370  Encoder Test  (PID = OFF)  "));
  Serial.println(F("  Gõ PWM -255..255 rồi nhấn Enter      "));
  Serial.println(F("  0 = dừng   |   r = reset encoder     "));
  Serial.println(F("========================================"));

  motor.motor_init(PIN_IN1, PIN_IN2, PIN_PWM, PIN_ENC_A, PIN_ENC_B, LEDC_CH);

  motor.enablePID(false); // PID tắt hoàn toàn
  motor.motor_stop();

  lastPrint = millis();
}

void loop() {
  // ── 1. Cập nhật encoder (cần để tính RPM, PID bị tắt nên không ảnh hưởng)
  motor.motor_update();

  // ── 2. Đọc lệnh Serial ────────────────────────────────────────────────────
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.equalsIgnoreCase("r")) {
      motor.encoder_reset();
      Serial.println(F(">> Encoder reset về 0"));
    } else {
      int pwm = input.toInt();
      pwm = constrain(pwm, -255, 255);
      currentPWM = pwm;
      motor.motor_run(pwm);

      Serial.print(F(">> PWM = "));
      Serial.println(pwm);
    }
  }

  // ── 3. In dữ liệu encoder định kỳ ────────────────────────────────────────
  uint32_t now = millis();
  if (now - lastPrint >= PRINT_INTERVAL_MS) {
    lastPrint = now;

    float rpm = motor.motor_get_speed();
    long enc = motor.encoder_get_count();

    Serial.print(F("PWM: "));
    Serial.print(currentPWM);
    Serial.print(F("  |  RPM: "));
    Serial.print(rpm, 1);
    Serial.print(F("  |  Enc: "));
    Serial.println(enc);
  }
}