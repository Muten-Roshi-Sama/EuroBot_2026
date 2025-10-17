// v1.0
#include <Arduino.h>
#include "FSM.h"


static FsmContext gFsm; 



void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("Setup.");
  
  fsmInit(gFsm);
    delay(2000);
}



void loop(){

  fsmStep(gFsm);
}








// void loop() {
//   // ============= TEST 1: AVANCER =============
//   Serial.println("\n>>> TEST 1: Avancer a vitesse 150");
//   movement.forward(150);
//   delay(2000);
//   movement.stop();
//   Serial.println("Arret");
//   delay(1000);
  
//   // ============= TEST 2: RECULER =============
//   Serial.println("\n>>> TEST 2: Reculer a vitesse 150");
//   movement.backward(150);
//   delay(2000);
//   movement.stop();
//   Serial.println("Arret");
//   delay(1000);
  
//   // ============= TEST 3: ROTATION SUR PLACE DROITE =============
//   Serial.println("\n>>> TEST 3: Rotation sur place vers la DROITE");
//   movement.rotateRight(120);
//   delay(1500);
//   movement.stop();
//   Serial.println("Arret");
//   delay(1000);
  
//   // ============= TEST 4: ROTATION SUR PLACE GAUCHE =============
//   Serial.println("\n>>> TEST 4: Rotation sur place vers la GAUCHE");
//   movement.rotateLeft(120);
//   delay(1500);
//   movement.stop();
//   Serial.println("Arret");
//   delay(1000);
  
//   // ============= TEST 5: VIRAGE DOUX DROITE =============
//   Serial.println("\n>>> TEST 5: Virage doux vers la droite");
//   movement.turnRightSoft(150);
//   delay(2000);
//   movement.stop();
//   Serial.println("Arret");
//   delay(1000);
  
//   // ============= TEST 6: VIRAGE DOUX GAUCHE =============
//   Serial.println("\n>>> TEST 6: Virage doux vers la gauche");
//   movement.turnLeftSoft(150);
//   delay(2000);
//   movement.stop();
//   Serial.println("Arret");
//   delay(1000);
  
//   // ============= TEST 7: AVANCER 30 CM (AVEC ENCODEURS) =============
//   Serial.println("\n>>> TEST 7: Avancer de 30 cm (bloquant)");
//   movement.moveDistance(30);
//   Serial.println("Arret");
//   delay(1000);
  
//   // ============= TEST 8: RECULER 20 CM (AVEC ENCODEURS) =============
//   Serial.println("\n>>> TEST 8: Reculer de 20 cm (bloquant)");
//   movement.moveDistance(-20);
//   Serial.println("Arret");
//   delay(1000);
  
//   // ============= TEST 9: ROTATION 90° DROITE (AVEC ENCODEURS) =============
//   Serial.println("\n>>> TEST 9: Rotation de 90 degres vers la droite (bloquant)");
//   movement.rotate(90);
//   Serial.println("Arret");
//   delay(1000);
  
//   // ============= TEST 10: ROTATION 90° GAUCHE (AVEC ENCODEURS) =============
//   Serial.println("\n>>> TEST 10: Rotation de 90 degres vers la gauche (bloquant)");
//   movement.rotate(-90);
//   Serial.println("Arret");
//   delay(1000);
  
//   Serial.println("\n===========================================");
//   Serial.println("   Tous les tests termines !");
//   Serial.println("   Redemarrage dans 5 secondes...");
//   Serial.println("===========================================\n");
//   delay(5000);
// }


// ============= EXEMPLE D'UTILISATION POUR LA FSM =============
/*
  Voici comment votre ami pourra utiliser les fonctions dans la FSM:
  
  // Mouvements simples (non-bloquants)
  movement.forward();              // Avance avec la vitesse par défaut
  movement.forward(200);           // Avance à vitesse 200
  movement.backward();             // Recule
  movement.stop();                 // Arrête
  
  // Rotations sur place (2 moteurs en sens opposé)
  movement.rotateLeft();           // Tourne sur place vers la gauche
  movement.rotateRight(180);       // Tourne sur place vers la droite à vitesse 180
  
  // Virages doux (une roue ralentit)
  movement.turnLeftSoft(150);      // Virage doux à gauche
  movement.turnRightSoft(150);     // Virage doux à droite
  
  // Mouvements précis avec encodeurs (BLOQUANTS - attendent la fin)
  movement.moveDistance(50);       // Avance de 50 cm puis s'arrête
  movement.moveDistance(-30);      // Recule de 30 cm puis s'arrête
  movement.rotate(90);             // Tourne de 90° vers la droite
  movement.rotate(-180);           // Tourne de 180° vers la gauche
  
  // Lecture des encodeurs
  long ticksL = movement.getLeftTicks();
  long ticksR = movement.getRightTicks();
  float distance = movement.getDistanceTraveled();
*/