// v0.0


//  TODO :
//        - Fix 
//        - Add 
//        -
//        - 

// v0.1 wiring FSM minimal usage
#if __has_include(<Arduino.h>)
#include <Arduino.h>
#else
// Minimal mock for non-Arduino analysis/linting
struct MockSerial {
  void begin(unsigned long) {}
  void println(const char*) {}
  explicit operator bool() const { return true; }
} Serial;
static inline void delay(unsigned long) {}
#endif
#include "C:/Users/felix/Documents/Master 1 ECAM/Embbeded_project/Projet_git/EuroBot_2026/lib/fsm/FSM.h"


static FsmContext gFsm; 

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("Setup.");
  
  fsmInit(gFsm);
}



void loop(){

  fsmStep(gFsm);
}




