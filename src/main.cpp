
// v1.0
#include <Arduino.h>
#include "FSM.h"
#include <Adafruit_MotorShield.h>
#include <Encoder.h>
#include "Movement.h"
#include "UltrasonicSensor.h"
#include "Switch.h"
#include "StartContact.h"
#include "EmergencyButton.h"

static FsmContext gFsm; 
// Changer la pin si nécessaire
Switch teamSwitch(5);
EmergencyButton emergencyButton(0);

Movement movement;
Encoder encoderLeft;
Encoder encoderRight;
UltrasonicSensor sonar(8, 9);  // TRIG = 8, ECHO = 9 (à adapter)

StartContact startContact(2);

unsigned long startTime = 0;
bool timerActive = false;


void setup() {
    Serial.begin(9600);

    movement.begin(
        6.0,   // diamètre roue
        10.7,  // wheelbase
        70,   // résolution encodeur
        2,     // pin encodeur gauche
        3,     // pin encodeur droit
        100    // vitesse
    );

    teamSwitch.begin(); // initialiser la pin en INPUT_PULLUP

    startContact.begin();

    delay(1000);
    Serial.println("Test mouvement...");


}

void loop() {

    // movement.rotate(80);
    // delay(1000);
    // movement.rotate(80);
    // delay(1000);
    // movement.rotate(80);
    // delay(1000);
  
    // delay(5000);
    // movement.moveDistance(100);

    // while (true);
    
    // Test moteur simple
    // movement.motorLeft->setSpeed(150);
    // movement.motorLeft->run(FORWARD);
    // delay(2000);
    // movement.motorLeft->run(RELEASE);
    // delay(500);

    // movement.motorRight->setSpeed(150);
    // movement.motorRight->run(FORWARD);
    // delay(2000);
    // movement.motorRight->run(RELEASE);
    // delay(500);

    // while(true);
   

    // if (startContact.isInserted()){
    //     Serial.println("Tirette EN PLACE → robot doit rester à l'arrêt");
    // }

    // else{
    //     Serial.println("Tirette RETIREE → robot PEUT démarrer !");

    //     //Démarrer le timer (tester avec ça, sinon ajouter dans les boucles des fonctions bloquantes comme avec le bouton d'arrêt d'urgence)
    //     if (!timerActive){
    //         Serial.println("Equipe 1");
    //         startTime = millis();
    //         timerActive = true;
    //     }

    //     if (teamSwitch.isOn()){
    //         Serial.println("Equipe 2");
    //         movement.moveDistance(100);
    //     }

    //     else{
    //         //TOURNER DE 90 DEGRÉS
    //         movement.rotate(90);
    //         delay(1000);
    //         movement.rotate(90);
    //         delay(1000);
    //         movement.rotate(90);
    //         delay(1000);
    //         Serial.println("Robot éteint");
    //     }
        
    // }
    
   

    // //RÉALISER UN CARRÉ DE 1 MÈTRE
    movement.moveDistance(25);
    delay(1000);
    movement.rotate(80);
    delay(1000);
    movement.moveDistance(25);
    delay(1000);
    movement.rotate(80);
    delay(1000);
    movement.moveDistance(25);
    delay(1000);
    movement.rotate(80);
    delay(1000);
    movement.moveDistance(25);
    while(true);
    

    // //DÉTECTION D'OBSTACLE (utilisation des fonctions moveDistanceStepped à confirmer, sinon repasser en moveDistance)
    // // si obstacle à moins de 20 cm → STOP + esquive
    // if (sonar.isObstacle(20.0)) {
    //     Serial.println("Obstacle détecté ! Arrêt + évitement");

    //     movement.stop();
    //     delay(200);

    //     // exemple : petite rotation et on repart
    //     movement.rotateLeft(30);
    //     delay(200);

    //     movement.moveDistanceStepped(10); // avance un peu
    //     delay(200);
    // }
    // else {
    //     // sinon on avance tout droit
    //     movement.moveDistanceStepped(5);
    // }


    // //MARQUER DES POINTS (AVANCER + FAIRE LEVER LE PLATEAU)


    // //TEST BOUTON D'ARRÊT D'URGENCE (ajouter fonctions qui lit l'état du bouton)



    // //TEST TIRETTE DE DÉMARRAGE + RESPECT DE LA DURÉE (FAIRE EN SORTE QUE LE ROBOT BOUGE ET S'ARRÊTE APRÈS 100 SECONDES, ajouter une notion de timer)

    // movement.stop();

    // Serial.print("Mouvement terminé");

    // while(true);
}





///// Version FSM

// void setup() {
//   Serial.begin(9600);
//   while (!Serial) {}
//   Serial.println("Setup.");
  
//   fsmInit(gFsm);
//     delay(2000);
    
// }


// void loop(){
//     Serial.println("Début FSM.");
//   fsmStep(gFsm);

//}


// #include <Arduino.h>
// #include "StartContact.h"

// StartContact startContact(5);  // par exemple sur la pin 3

// void setup() {
//     Serial.begin(9600);
//     startContact.begin();

//     Serial.println("Start Contact test ready");
// }

// void loop() {

//     if (startContact.isInserted()) {
//         Serial.println("Tirette EN PLACE → robot doit rester à l'arrêt");
//     } else {
//         Serial.println("Tirette RETIREE → robot PEUT démarrer !");
//     }

//     delay(300);
// }









