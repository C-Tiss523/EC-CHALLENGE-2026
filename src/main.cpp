#include <Arduino.h>
#include <LineBot.h>

void setup() {
  LineBot::begin();
}

void loop() {
  LineBot::update();
}
