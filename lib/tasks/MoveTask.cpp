#include "MoveTask.h"
#include "settings.h"
#include "isr_flags.h"

// Helper: absolute long
static inline long labs_long(long v) { return v < 0 ? -v : v; }

void MoveTask::start(Movement &mv) {
    started = true;
    startMs = millis();
    paused = false;
    finished = false;
    cancelled = false;

    // 1. Reset
    mv.resetEncoders();

    // 2. Calcul des cibles
    if (mode == MoveTaskMode::MOVE_DISTANCE) {
        targetTicks = labs_long(mv.cmToTicks(value));
    } else { // ROTATE
        targetTicks = labs_long(mv.degreesToTicks(value));
    }

    // 3. Init PID
    persistentError = 0.0f;
    integral = 0.0f;
    loopCounter = 0;
    lastPidLoopMs = millis(); 
}

void MoveTask::update(Movement &mv) {
    if (cancelled || finished || paused) return;

    // Timeout global
    if (timeoutMs && (millis() - startMs) > timeoutMs) {
        mv.stop();
        cancelled = true;
        finished = true;
        return;
    }

    // --- GESTION DU TEMPS (NON-BLOQUANT) ---
    // On exécute la logique PID seulement toutes les X ms (ex: 10ms)
    // C'est ce qui remplace ton delay(MOVEMENT_LOOP_DELAY)
    if (millis() - lastPidLoopMs < MOVEMENT_LOOP_DELAY) {
        return; 
    }
    lastPidLoopMs = millis(); // Reset du timer PID

    // --- LOGIQUE PI (Ton code adapté) ---
    long leftTicks  = mv.getLeftTicks();
    long rightTicks = mv.getRightTicks();

    // Vérification de fin
    if (abs(leftTicks) >= targetTicks || abs(rightTicks) >= targetTicks) {
        mv.stop();
        finished = true;
        return;
    }

    float dt = MOVEMENT_LOOP_DELAY / 1000.0f;
    float error = leftTicks - rightTicks;
    
    // Erreur persistante
    persistentError = persistentError + error;

    // Intégration + Anti-windup + Deadzone
    if (abs(persistentError) > deadzone) {
        integral += persistentError * dt;
    }

    if (integral > integralMax) integral = integralMax;
    if (integral < -integralMax) integral = -integralMax;

    // Correction
    float correction = Kp * persistentError + Ki * integral;

    // Calcul des vitesses
    int baseSpeed = getSpeed() ? getSpeed() : mv.defaultSpeed;
    int leftSpeed = 0;
    int rightSpeed = 0;

    // --- WARM-UP ---
    if (loopCounter < warmupIterations) {
        // Pendant les 50 premiers cycles, on n'envoie rien aux moteurs
        // mais on laisse le PID "écouter" l'erreur qui s'accumule
        mv.setRawSpeeds(0, 0); 
        
        // Debug optionnel
        // Serial.print("[Warm-up] Err: "); Serial.println(error);
    } 
    else {
        // --- PID ACTIF ---
        // Application de la correction
        // Note: La logique ici suppose que correction positive = gauche trop rapide / droite trop lent
        // Adapte les signes +/- si ton robot tourne du mauvais côté
        leftSpeed  = constrain(baseSpeed - (correction/2), 90, 255);
        rightSpeed = constrain(baseSpeed + (correction/2), 90, 255);

        // Direction globale
        bool goForward = (value >= 0);
        
        // Si on doit reculer, on inverse les vitesses pour setRawSpeeds
        if (!goForward) {
            leftSpeed = -leftSpeed;
            rightSpeed = -rightSpeed;
        }

        // Envoi aux moteurs via la méthode créée à l'étape 1
        mv.setRawSpeeds(leftSpeed, rightSpeed);

        // Debug
        Serial.print("Err: "); Serial.print(error);
        Serial.print(" | P_Err: "); Serial.print(persistentError);
        Serial.print(" | L: "); Serial.print(leftSpeed);
        Serial.print(" | R: "); Serial.println(rightSpeed);
    }

    loopCounter++;
    mv.updateEncoderTimestamps();
}

TaskInterruptAction MoveTask::handleInterrupt(Movement &mv, uint8_t isrFlags) {
    if (isrFlags & ISR_FLAG_EMERGENCY) {
        mv.stop();
        cancelled = true;
        finished = true;
        return TaskInterruptAction::CANCEL;
    }
    if (isrFlags & ISR_FLAG_OBSTACLE) {
        mv.stop();
        paused = true;
        return TaskInterruptAction::PAUSE;
    }
    if (isrFlags & ISR_FLAG_OBSTACLE_CLEARED) {
        if (paused) {
            // Reprendre
            paused = false;
            lastPidLoopMs = millis(); // Important pour ne pas fausser le PID
            return TaskInterruptAction::HANDLE;
        }
    }
    return TaskInterruptAction::IGNORE;
}

void MoveTask::cancel(Movement &mv) {
    cancelled = true;
    mv.stop();
    finished = true;
}

void MoveTask::resume(Movement &mv) {
    if (!paused || cancelled || finished) return;
    paused = false;
    lastPidLoopMs = millis();
}