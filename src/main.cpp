
// // v1.0
// #include <Arduino.h>
// #include "FSM.h"
// #include <Adafruit_MotorShield.h>
// #include <Encoder.h>


// static FsmContext gFsm; 



// void setup() {
//   Serial.begin(9600);
//   while (!Serial) {}
//   Serial.println("Setup.");
  
//   fsmInit(gFsm);
//     delay(2000);
// }



// void loop(){

//   fsmStep(gFsm);
// }


#include <Arduino.h>
#include "capteur_lidar.h"

// void setup() {
//   Serial.begin(9600);
//   Serial.println("Initialisation du capteur LiDAR...");

//   if (!initLidar()) {
//     Serial.println("ERREUR: Capteur non detecte!");
//     Serial.println("Verifiez les connexions I2C");
//     while (1) {
//       delay(1000);
//     }
//   }

//   Serial.println("Capteur initialise!");
//   Serial.println("\nDistance | Etat");
//   Serial.println("---------|------");
// }

// void loop() {
//   int distance = lireDistance();
//   afficherEtatObstacle(distance);

//   delay(500);
// }


void setup() {
  Serial.begin(9600);
  Serial.println("Initialisation du capteur LiDAR...");

  if (!initLidar()) {
    Serial.println("ERREUR: Capteur non detecte!");
    Serial.println("Verifiez les connexions I2C");
    while (1) { delay(1000); }
  }

  Serial.println("Capteur initialise!");
  Serial.println("Robot test : avancer / stop selon distance\n");
}

void loop() {
  int distance = lireDistance();

  Serial.print("Distance : ");
  Serial.print(distance);
  Serial.print(" mm → ");

  if (distance < 150) { 
    Serial.println("⚠️ OBSTACLE — STOP");
  } else {
    Serial.println("OK — avancer");
  }

  delay(300);
}