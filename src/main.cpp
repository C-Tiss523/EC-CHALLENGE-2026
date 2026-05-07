#include <Arduino.h>
#include "encoder.h"

// ===== SỬA 4 CHÂN NÀY THEO MẠCH CỦA BẠN =====
#define ENC_L_A  36
#define ENC_L_B  39

#define ENC_R_A  34
#define ENC_R_B  35

// Nếu chưa biết countsPerRev thì để tạm 1 cũng được
// Vì mình đang cần đọc raw count là chính
#define TEMP_COUNTS_PER_REV 1

Encoder encL;
Encoder encR;

unsigned long lastPrint = 0;
long lastL = 0;
long lastR = 0;

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("===== ENCODER RAW COUNT TEST =====");
  Serial.println("Nhap 'r' de reset count ve 0");
  Serial.println("Quay tay tung banh 1 vong roi xem count thay doi bao nhieu");
  Serial.println();

  encL.encoder_init(ENC_L_A, ENC_L_B, TEMP_COUNTS_PER_REV);
  encR.encoder_init(ENC_R_A, ENC_R_B, TEMP_COUNTS_PER_REV);

  encL.encoder_reset();
  encR.encoder_reset();

  lastL = encL.encoder_get_count();
  lastR = encR.encoder_get_count();
}

void loop() {
  // Nhập r trên Serial Monitor để reset count
  if (Serial.available()) {
    char c = Serial.read();

    if (c == 'r' || c == 'R') {
      encL.encoder_reset();
      encR.encoder_reset();

      lastL = 0;
      lastR = 0;

      Serial.println();
      Serial.println("[RESET] Encoder count = 0");
      Serial.println();
    }
  }

  // In count mỗi 200ms
  if (millis() - lastPrint >= 200) {
    lastPrint = millis();

    long countL = encL.encoder_get_count();
    long countR = encR.encoder_get_count();

    long deltaL = countL - lastL;
    long deltaR = countR - lastR;

    lastL = countL;
    lastR = countR;

    Serial.print("L=");
    Serial.print(countL);
    Serial.print(" | dL=");
    Serial.print(deltaL);

    Serial.print(" || R=");
    Serial.print(countR);
    Serial.print(" | dR=");
    Serial.print(deltaR);

    Serial.print(" || ABS_AVG=");
    Serial.println((labs(countL) + labs(countR)) / 2);
  }
}