#include <Wire.h>
#include <VL53L0X.h>

VL53L0X lox;

void setup() {
  Serial.begin(115200);
  Wire.begin(22, 23);

  if (!lox.init()) {
    Serial.println("Failed to detect VL53L0X");
    while (1);
  }

  lox.setTimeout(50);
  lox.startContinuous();
}

void loop() {
  Serial.println(lox.readRangeContinuousMillimeters());
  delay(50);
}
