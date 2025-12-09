#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

// Classe pour gérer un encodeur (roue ou moteur)
// Permet de compter les ticks, calculer la vitesse et les tours effectués
class Encoder {
private:
<<<<<<< HEAD
    volatile long tick = 0;                
    unsigned long timestamp = 0; 
    unsigned long tickInterval = 0; 
    int resolution = 1;

public:
    Encoder();                        
    Encoder(int resolution);          
    void addTick();
    void subtractTick();                 
    long getTicks();                   
    void changeResolution(int newResolution); 
    void setTimestamp(unsigned long time);   
    unsigned long getTimestamp();    
    void setTickInterval(unsigned long time); 
    unsigned long getTickInterval();  
    void reset();
    float getRevolutions(); 
    float getRPM();
=======
    volatile int tick = 0;                // Compteur de ticks (impulsions), modifié par ISR
    volatile unsigned long timestamp = 0; // Temps du dernier tick (micros)
    volatile unsigned long tickInterval = 0; // Intervalle de temps entre deux ticks
    int resolution = 20;                  // Nombre de ticks par révolution complète

public:
    Encoder();                            // Constructeur par défaut
    Encoder(int resolution);              // Constructeur avec résolution personnalisée
    
    void addTick();                        // Incrémente le compteur de ticks (roue avant)
    void subtractTick();                   // Décrémente le compteur (roue arrière)
    int getTicks();                        // Retourne le nombre de ticks actuels
    
    void changeResolution(int newResolution); // Modifie la résolution de l’encodeur
    void setTimestamp(unsigned long time);    // Définit le timestamp actuel
    unsigned long getTimestamp();             // Récupère le timestamp actuel
    void setTickInterval(unsigned long time); // Définit l’intervalle entre ticks
    unsigned long getTickInterval();          // Récupère l’intervalle entre ticks
    
    void reset();                            // Réinitialise tous les compteurs et timestamps
    float getRevolutions();                  // Retourne le nombre de tours complets
    float getRPM();                           // Retourne la vitesse en tours par minute
>>>>>>> feature/detection
};

#endif
