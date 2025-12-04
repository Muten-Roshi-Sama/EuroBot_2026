

//  TODO :
//        - Fix 
//        - Add 
//        -
//        - 

// v0.1 wiring FSM minimal usage

#include <Arduino.h>
#include <ArduinoJson.h>
#include "..\lib\comms\protocol.h"


void setup() {
  Serial.begin(115200);
  protocol::init(Serial, 115200);
}

void loop() {
  protocol::log("Looping...");
  delay(1000);
}




