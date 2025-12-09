#include "Encoder.h"

// Constructeur par défaut
Encoder::Encoder() {}

// Constructeur avec résolution personnalisée
Encoder::Encoder(int resolution) {
    this->resolution = resolution; // Stocke la résolution spécifiée
}

// Ajoute 1 tick (appelé généralement par l’ISR quand la roue avance)
void Encoder::addTick() {
    tick++;  // Incrémente le compteur
}

<<<<<<< HEAD
long Encoder::getTicks() {
=======
// Retire 1 tick (appelé si la roue recule)
void Encoder::subtractTick() {
    tick--;  // Décrémente le compteur
}

// Retourne le nombre de ticks accumulés
int Encoder::getTicks() {
>>>>>>> feature/detection
    return tick;
}

// Change la résolution de l’encodeur
void Encoder::changeResolution(int newResolution) {
    resolution = newResolution;
}

// Définit le timestamp actuel (en microsecondes)
void Encoder::setTimestamp(unsigned long time) {
    timestamp = time;
}

// Retourne le timestamp actuel
unsigned long Encoder::getTimestamp() {
    return timestamp;
}

// Définit l’intervalle de temps entre deux ticks (en microsecondes)
void Encoder::setTickInterval(unsigned long time) {
    tickInterval = time;
}

// Retourne l’intervalle entre ticks
unsigned long Encoder::getTickInterval() {
    return tickInterval;
}

// Réinitialise tous les compteurs et timestamps
void Encoder::reset() {
    tick = 0;
    timestamp = 0;
    tickInterval = 0;
}

// Calcule le nombre de révolutions complètes depuis le départ
float Encoder::getRevolutions() {
    return (float)tick / resolution; // ticks / ticks_par_tour
}

// Calcule la vitesse en RPM
float Encoder::getRPM() {
    if (tickInterval == 0) return 0.0; // éviter division par zéro

    // Formule :
    // RPM = (1 tour / durée tour en minutes)
    // tickInterval = temps pour 1 tick
    // resolution = nombre de ticks par tour
    // donc temps pour 1 tour = tickInterval * resolution
    // conversion microsecondes -> minutes : 60 * 1e6
    return (60.0 * 1e6) / (tickInterval * resolution);
}
