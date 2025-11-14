#include <Arduino.h>
#include <Adafruit_VL53L0X.h>

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {  
    Serial.begin(9600);

    // Attendre l'ouverture du port série (utile pour cartes USB natives)
    while (!Serial) {
        delay(1);
    }
  
    Serial.println("Adafruit VL53L0X test");

    if (!lox.begin()) {
        Serial.println(F("Failed to boot VL53L0X"));
        while (1);
    }

    Serial.println(F("VL53L0X API Simple Ranging example\n\n")); 
}

void loop() {
    VL53L0X_RangingMeasurementData_t measure;

    Serial.print("Reading a measurement... ");
    lox.rangingTest(&measure, false); // 'true' = debug

    if (measure.RangeStatus != 4) {  // 4 = out of range
        Serial.print("Distance (mm): ");
        Serial.println(measure.RangeMilliMeter);
    } else {
        Serial.println("Out of range");
    }

    delay(100);
}
