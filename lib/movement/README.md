# 🤖 Système de Mouvement - EuroBot 2026

## 📋 Vue d'ensemble

Ce module fournit une interface complète pour contrôler les mouvements d'un robot différentiel à 2 roues avec encodeurs. Conçu pour être utilisé avec une machine à états (FSM).

## 🔧 Configuration matérielle

- **Microcontrôleur**: Arduino Uno
- **Shield moteur**: Adafruit Motor Shield V2
- **Moteurs**: 2 moteurs DC (Motor 1 = gauche, Motor 2 = droite)
- **Encodeurs**: 2 encodeurs rotatifs sur pins d'interruption (2 et 3)

## ⚙️ Initialisation

### Dans `setup()`

```cpp
#include "Movement.h"

// Paramètres physiques du robot (À MESURER PRÉCISÉMENT!)
const float WHEEL_DIAMETER = 6.5;      // Diamètre roue en cm
const float WHEEL_BASE = 15.0;         // Distance entre les roues en cm
const int ENCODER_RESOLUTION = 20;     // Ticks par tour
const int ENCODER_PIN_LEFT = 2;        // Pin encodeur gauche
const int ENCODER_PIN_RIGHT = 3;       // Pin encodeur droite
const int DEFAULT_SPEED = 150;         // Vitesse par défaut (0-255)

Movement movement;

void setup() {
    Serial.begin(9600);
    
    // Initialisation du système
    movement.begin(WHEEL_DIAMETER, WHEEL_BASE, ENCODER_RESOLUTION, 
                   ENCODER_PIN_LEFT, ENCODER_PIN_RIGHT, DEFAULT_SPEED);
}
```

## 📚 Fonctions disponibles

### 🚀 Mouvements basiques (non-bloquants)

Ces fonctions démarrent le mouvement et retournent immédiatement. Le robot continue jusqu'à appel de `stop()`.

```cpp
movement.forward();           // Avance (vitesse par défaut)
movement.forward(200);        // Avance à vitesse 200
movement.backward();          // Recule
movement.backward(150);       // Recule à vitesse 150
movement.stop();              // Arrête les moteurs
```

**Utilisation typique:**
```cpp
movement.forward(180);
delay(2000);    // Avance pendant 2 secondes
movement.stop();
```

---

### 🔄 Rotations sur place

Les deux moteurs tournent en sens opposé pour une rotation sur place.

```cpp
movement.rotateLeft();        // Rotation gauche (vitesse défaut)
movement.rotateLeft(180);     // Rotation gauche vitesse 180
movement.rotateRight();       // Rotation droite (vitesse défaut)
movement.rotateRight(180);    // Rotation droite vitesse 180
```

**Utilisation typique:**
```cpp
movement.rotateRight(150);
delay(1000);    // Tourne pendant 1 seconde
movement.stop();
```

---

### 🌊 Virages doux

Une roue ralentit pour effectuer un virage en courbe.

```cpp
movement.turnLeftSoft(150);   // Virage doux à gauche
movement.turnRightSoft(150);  // Virage doux à droite
```

---

### 📏 Mouvements précis (BLOQUANTS)

Ces fonctions attendent la fin du mouvement avant de retourner. Utilisent les encodeurs pour une précision maximale.

```cpp
movement.moveDistance(50);    // Avance de 50 cm
movement.moveDistance(-30);   // Recule de 30 cm
movement.rotate(90);          // Tourne de 90° à droite
movement.rotate(-180);        // Tourne de 180° à gauche
```

**⚠️ Important:** Ces fonctions bloquent l'exécution jusqu'à la fin du mouvement!

**Utilisation typique:**
```cpp
movement.moveDistance(50);    // Attend que le robot ait avancé de 50 cm
// Code suivant exécuté après la fin du mouvement
Serial.println("Mouvement termine!");
```

---

### 📊 Lecture des encodeurs

```cpp
long ticksLeft = movement.getLeftTicks();
long ticksRight = movement.getRightTicks();
float distance = movement.getDistanceTraveled();
```

---

## 💡 Exemples d'utilisation

### Exemple 1: Carré simple

```cpp
void loop() {
    // Avance de 50 cm
    movement.moveDistance(50);
    delay(500);
    
    // Tourne de 90°
    movement.rotate(90);
    delay(500);
    
    // Répéter 4 fois = carré complet
}
```

### Exemple 2: Mouvement continu avec capteur

```cpp
void loop() {
    // Avance jusqu'à détection d'obstacle
    movement.forward(150);
    
    while (digitalRead(CAPTEUR_PIN) == HIGH) {
        // Continue d'avancer
        delay(10);
    }
    
    // Obstacle détecté
    movement.stop();
    movement.moveDistance(-20);  // Recule de 20 cm
    movement.rotate(90);         // Tourne de 90°
}
```

### Exemple 3: Intégration avec FSM

```cpp
enum State { IDLE, MOVING, TURNING, DONE };
State currentState = IDLE;

void loop() {
    switch (currentState) {
        case IDLE:
            currentState = MOVING;
            break;
            
        case MOVING:
            movement.moveDistance(30);  // Bloquant
            currentState = TURNING;
            break;
            
        case TURNING:
            movement.rotate(90);        // Bloquant
            currentState = DONE;
            break;
            
        case DONE:
            movement.stop();
            delay(2000);
            currentState = IDLE;
            break;
    }
}
```

---

## 🎯 Conseils pour la FSM

### ✅ Bonnes pratiques

1. **Fonctions bloquantes** → Utilisez dans les transitions d'états
   ```cpp
   case STATE_A:
       movement.moveDistance(50);  // Bloquant
       currentState = STATE_B;     // Exécuté après la fin
       break;
   ```

2. **Fonctions non-bloquantes** → Utilisez avec surveillance de capteurs
   ```cpp
   case STATE_B:
       movement.forward(150);
       if (capteurDetecteObstacle()) {
           movement.stop();
           currentState = STATE_C;
       }
       break;
   ```

3. **Toujours arrêter** avant de changer de direction
   ```cpp
   movement.forward(200);
   delay(1000);
   movement.stop();         // Important!
   delay(100);              // Petit délai
   movement.backward(200);
   ```

### ❌ À éviter

```cpp
// ❌ MAL: Pas de stop() entre les mouvements
movement.forward(200);
movement.backward(200);  // Conflit!

// ✅ BIEN:
movement.forward(200);
delay(1000);
movement.stop();
delay(100);
movement.backward(200);
```

---

## 📐 Calibration

Pour des mouvements précis, mesurez avec précision:

1. **Diamètre des roues**: Mesurez avec un pied à coulisse
2. **Distance entre roues**: Distance entre les centres des roues
3. **Résolution encodeur**: Comptez les ticks sur un tour complet
4. **Test de calibration**:
   ```cpp
   movement.moveDistance(100);  // Demande 100 cm
   // Mesurez la distance réelle parcourue
   // Ajustez WHEEL_DIAMETER si nécessaire
   ```

---

## 🐛 Dépannage

| Problème | Solution |
|----------|----------|
| Robot ne bouge pas | Vérifier alimentation moteurs et connexions Motor Shield |
| Distance imprécise | Calibrer WHEEL_DIAMETER et ENCODER_RESOLUTION |
| Rotation imprécise | Calibrer WHEEL_BASE (distance entre roues) |
| Encodeurs ne comptent pas | Vérifier connexions sur pins 2 et 3 (interruptions) |
| Robot tourne à gauche en avançant | Moteurs pas alignés ou vitesses différentes |

---

## 📄 Fichiers

- `lib/movement/Movement.h` - En-tête de la classe
- `lib/movement/Movement.cpp` - Implémentation
- `src/main.cpp` - Programme de test complet
- `examples/simple_fsm_example.cpp` - Exemple FSM

---

## 👥 Auteurs

- **Mouvement** - Vass
- **FSM** - Votre ami

Bon courage pour Eurobot 2026! 🏆
