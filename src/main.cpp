#include <Arduino.h>
#include "drive.h"

RobotDrive bot;

bool done = false;

void setup() {
  Serial.begin(115200);
  bot.begin();

  bot.setTurnCalib(0.9f, 5);
}

void loop() {
  bot.update();

  if (!done) {
    delay(1000);

    bot.turnAngle(30, 120);    // quay phải 90 độ
    delay(1000);

    bot.turnAngle(-90, 120);   // quay trái 90 độ
    delay(1000);

    bot.stop();
    done = true;
  }
  while(1);
}