#include <Arduino.h>
#include "LineBot.h"

// =====================================================
// Nhiệm vụ 3 – Dò line (chỉ dùng thư viện LineBot)
//
// Phần cứng do LineBot::begin() tự khởi tạo:
//   Motor  : TB6612FNG  (AIN1/2=25/33, BIN1/2=26/27, PWMA=32, PWMB=14)
//   Sensor : 8 IR qua HC4067 MUX (S0=23, S1=19, S2=18, SIG=13)
//   OLED   : SSD1306 128x64 I2C  (SDA=21, SCL=22)
//   Nút    : GPIO 2 ACTIVE-HIGH
//              Lần 1 → calib nền TRẮNG (500 mẫu)
//              Lần 2 → calib LINE ĐEN  (500 mẫu)
//              Sau đó → toggle RUN / STOP
//   BLE    : "LineBot" – chỉnh Kp/Ki/Kd từ dashboard
// =====================================================
void nv3() {
  LineBot::update();
}

void setup() {
  LineBot::begin();
}

void loop() {
  nv3();
}