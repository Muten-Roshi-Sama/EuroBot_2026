#include "MoveTask.h"
#include "settings.h"
#include "isr_flags.h"
#include "../util/Debug.h"
#include <Arduino.h> 

// Helper: absolute long pour les ticks
static inline long labs_long(long v) { return v < 0 ? -v : v; }


// ============ MOVE TASK ==================

void MoveTask::start(Movement &mv) {
    started = true;
    startMs = millis();
    paused = false;
    finished = false;
    cancelled = false;

    mv.resetEncoders();

    // PID/internal init
    integralError = 0.0f;
    loopCounter = 0;
    lastPidLoopMs = millis();

    // Tunables / defaults for distance
    Kp = 0.8f;
    Ki = 0.0f;
    warmupIterations = 30;
    baseSpeed = getSpeed() ? getSpeed() : mv.defaultSpeed;
    minSpeed = 110;
    maxSpeed = 255;
    deadZone = 0.0f;

    targetTicks = labs_long(mv.cmToTicks(distanceCm));

    // Start moving slowly; update() will ramp and sync wheels
    if (distanceCm >= 0.0f) mv.forward(minSpeed); else mv.backward(minSpeed);

    debugPrintf(DBG_MOVEMENT, "MoveTask START dist=%.1fcm tgt=%ld base=%d min=%d Kp=%.2f",
            distanceCm, targetTicks, baseSpeed, minSpeed, Kp);
}

void MoveTask::update(Movement &mv) {
    if (cancelled || finished || paused) return;

    if (timeoutMs && (millis() - startMs) > timeoutMs) {
        mv.stop();
        cancelled = true;
        finished = true;
        debugPrintf(DBG_MOVEMENT, "MoveTask TIMEOUT after %lums", millis() - startMs);
        return;
    }

    if (millis() - lastPidLoopMs < MOVEMENT_LOOP_DELAY) return;
    float dt = (millis() - lastPidLoopMs) / 1000.0f;
    lastPidLoopMs = millis();

    long leftTicks, rightTicks;
    noInterrupts();
    leftTicks = mv.getLeftTicks();
    rightTicks = mv.getRightTicks();
    interrupts();

    long progressTicks = max(labs_long(leftTicks), labs_long(rightTicks));
    if (progressTicks >= targetTicks) {
        mv.stop();
        finished = true;
        debugPrintf(DBG_MOVEMENT, "MoveTask DONE %ld/%ld loops=%d", progressTicks, targetTicks, loopCounter);
        return;
    }

    // Sync error (left - right)
    float error = (float)(leftTicks - rightTicks);
    integralError += error * Ki * dt;
    float correction = Kp * error + integralError;
    float corrPWM = correction * 0.3f;

    // Warmup ramp for base speed
    float rampFactor = 1.0f;
    if (loopCounter < warmupIterations && warmupIterations > 0) {
        rampFactor = (float)loopCounter / (float)warmupIterations;
        if (rampFactor < 0.05f) rampFactor = 0.05f;
    }
    int targetBase = minSpeed + (int)((baseSpeed - minSpeed) * rampFactor);

    int leftPWM = (int)constrain((float)targetBase - corrPWM, (float)minSpeed, 255.0f);
    int rightPWM = (int)constrain((float)targetBase + corrPWM, (float)minSpeed, 255.0f);

    if (distanceCm >= 0.0f) {
        mv.motorLeft->setSpeed(leftPWM);
        mv.motorRight->setSpeed(rightPWM);
        mv.motorLeft->run(FORWARD);
        mv.motorRight->run(FORWARD);
    } else {
        mv.motorLeft->setSpeed(leftPWM);
        mv.motorRight->setSpeed(rightPWM);
        mv.motorLeft->run(BACKWARD);
        mv.motorRight->run(BACKWARD);
    }

    // Debug: concise runtime diagnostics (keep under buffer)
    debugPrintf(DBG_MOVEMENT, "MV U: L=%ld R=%ld prog=%ld/%ld err=%.1f cPWM=%.1f base=%d Lp=%d Rp=%d",
                leftTicks, rightTicks, progressTicks, targetTicks, error, corrPWM, targetBase, leftPWM, rightPWM);


    loopCounter++;
    mv.updateEncoderTimestamps();
    }

TaskInterruptAction MoveTask::handleInterrupt(Movement &mv, uint8_t isrFlags) {
    debugPrintf(DBG_MOVEMENT, "MoveTask ISR flags=0x%02X", isrFlags);
    if (isrFlags & ISR_FLAG_EMERGENCY) {
        mv.stop();
        cancelled = true;
        finished = true;
        debugPrintf(DBG_MOVEMENT, "MoveTask ISR: EMERGENCY -> CANCEL");
        return TaskInterruptAction::CANCEL;
    }
    if (isrFlags & ISR_FLAG_OBSTACLE) {
        mv.stop();
        paused = true;
        debugPrintf(DBG_MOVEMENT, "MoveTask ISR: OBSTACLE -> PAUSE");
        return TaskInterruptAction::PAUSE;
    }
    if (isrFlags & ISR_FLAG_OBSTACLE_CLEARED && paused) {
        paused = false;
        lastPidLoopMs = millis();
        debugPrintf(DBG_MOVEMENT, "MoveTask ISR: OBSTACLE_CLEARED -> RESUME");
        return TaskInterruptAction::HANDLE;
    }
    return TaskInterruptAction::IGNORE;
}

void MoveTask::cancel(Movement &mv) {
    cancelled = true;
    mv.stop();
    finished = true;
    debugPrintf(DBG_MOVEMENT, "MoveTask CANCELLED");
}

void MoveTask::resume(Movement &mv) {
    if (!paused || cancelled || finished) {
        debugPrintf(DBG_MOVEMENT, "MoveTask resume ignored paused=%d cancelled=%d finished=%d", paused, cancelled, finished);
        return;
    }
    paused = false;
    lastPidLoopMs = millis();
    debugPrintf(DBG_MOVEMENT, "MoveTask RESUMED");
}


// ============ ROTATE TASK ==================
void RotateTask::start(Movement &mv) {
    started = true;
    startMs = millis();
    paused = false;
    finished = false;
    cancelled = false;

    mv.resetEncoders();

    integralError = 0.0f;
    loopCounter = 0;
    lastPidLoopMs = millis();

    // Rotation tunables
    Kp = 1.3f;
    Ki = 0.08f;
    minSpeed = 50;
    maxSpeed = 90;
    deadZone = 3.5f;

    debugPrintf(DBG_MOVEMENT, "RotateTask START deg=%.1f Kp=%.2f Ki=%.2f dead=%0.1f", angleDeg, Kp, Ki, deadZone);
}

void RotateTask::update(Movement &mv) {
    if (cancelled || finished || paused) return;

    if (timeoutMs && (millis() - startMs) > timeoutMs) {
        mv.stop();
        cancelled = true;
        finished = true;
        debugPrintf(DBG_MOVEMENT, "RotateTask TIMEOUT after %lums", millis() - startMs);
        return;
    }

    if (millis() - lastPidLoopMs < MOVEMENT_LOOP_DELAY) return;
    float dt = (millis() - lastPidLoopMs) / 1000.0f;
    lastPidLoopMs = millis();

    long leftTicks, rightTicks;
    noInterrupts();
    leftTicks = mv.getLeftTicks();
    rightTicks = mv.getRightTicks();
    interrupts();

    long avgTicks = (labs_long(leftTicks) + labs_long(rightTicks)) / 2;
    float currentAngle = mv.ticksToDegrees(avgTicks);

    float targetAngleAbs = abs(angleDeg);
    float error = targetAngleAbs - currentAngle;

    if (abs(error) <= deadZone) {
        mv.stop();
        finished = true;
        debugPrintf(DBG_MOVEMENT, "RotateTask DONE ang=%.2f tgt=%.1f", currentAngle, targetAngleAbs);
        return;
    }

    integralError += error * dt;
    // basic anti-windup
    if (integralError > 50.0f) integralError = 50.0f;
    if (integralError < -50.0f) integralError = -50.0f;

    float pidOutput = (Kp * error) + (Ki * integralError);
    int speed = (int)constrain(abs(pidOutput), (float)minSpeed, (float)maxSpeed);

    bool originalDirectionIsRight = (angleDeg >= 0.0f);
    bool shouldGoOriginalWay = (error > 0.0f);
    bool goRight = (originalDirectionIsRight == shouldGoOriginalWay);

    if (goRight) {
        mv.motorLeft->setSpeed(speed);
        mv.motorRight->setSpeed(speed);
        mv.motorLeft->run(FORWARD);
        mv.motorRight->run(BACKWARD);
    } else {
        mv.motorLeft->setSpeed(speed);
        mv.motorRight->setSpeed(speed);
        mv.motorLeft->run(BACKWARD);
        mv.motorRight->run(FORWARD);
    }

    debugPrintf(DBG_MOVEMENT, "ROT U: ang=%.1f cur=%.2f err=%.2f pid=%.2f spd=%d dir=%s",
                targetAngleAbs, currentAngle, error, pidOutput, speed, goRight ? "R" : "L");

    loopCounter++;
    mv.updateEncoderTimestamps();
}

TaskInterruptAction RotateTask::handleInterrupt(Movement &mv, uint8_t isrFlags) {
    if (isrFlags & ISR_FLAG_EMERGENCY) {
        mv.stop();
        cancelled = true;
        finished = true;
        debugPrintf(DBG_MOVEMENT, "RotateTask ISR: EMERGENCY -> CANCEL");
        return TaskInterruptAction::CANCEL;
    }
    if (isrFlags & ISR_FLAG_OBSTACLE) {
        mv.stop();
        paused = true;
        debugPrintf(DBG_MOVEMENT, "RotateTask ISR: OBSTACLE -> PAUSE");
        return TaskInterruptAction::PAUSE;
    }
    if (isrFlags & ISR_FLAG_OBSTACLE_CLEARED && paused) {
        paused = false;
        lastPidLoopMs = millis();
        debugPrintf(DBG_MOVEMENT, "RotateTask ISR: OBSTACLE_CLEARED -> RESUME");
        return TaskInterruptAction::HANDLE;
    }
    return TaskInterruptAction::IGNORE;
}

void RotateTask::cancel(Movement &mv) {
    cancelled = true;
    mv.stop();
    finished = true;
    debugPrintf(DBG_MOVEMENT, "RotateTask CANCELLED");
}

void RotateTask::resume(Movement &mv) {
        if (!paused || cancelled || finished) return;
        paused = false;
        lastPidLoopMs = millis();
        debugPrintf(DBG_MOVEMENT, "RotateTask RESUMED");
}
















// void MoveTask::start(Movement &mv) {
//     started = true;
//     startMs = millis();
//     paused = false;
//     finished = false;
//     cancelled = false;

//     // Reset Hardware
//     mv.resetEncoders();

//     // Reset PID Memory
//     integralError = 0.0f;
//     loopCounter = 0;
//     lastPidLoopMs = millis();

//     // --- CONFIGURATION SELON LE MODE ---
//     if (mode == MoveTaskMode::MOVE_DISTANCE) {
//         // --- Paramètres pour MOVE DISTANCE (Synchronisation) ---
//         Kp = 0.8f; 
//         Ki = 0.0f;
//         warmupIterations = 30;
//         baseSpeed = getSpeed() ? getSpeed() : mv.defaultSpeed;
//         minSpeed = 110;
//         maxSpeed = 255;
//         deadZone = 0.0f; // Non utilisé en distance pure ici

//         targetTicks = labs_long(mv.cmToTicks(value));
        
//         // Lancement initial (Warmup gérera la rampe)
//         // On prépare juste la direction, la vitesse sera mise dans update
//         if (value >= 0.0f) mv.forward(minSpeed); else mv.backward(minSpeed);

//         debugPrintf(DBG_MOVEMENT, "Start DIST: Val=%.1fcm Ticks=%ld Kp=%.2f", value, targetTicks, Kp);

//     } else {
//         // --- Paramètres pour ROTATION (Cible Position) ---
//         Kp = 1.3f; 
//         Ki = 0.08f;
//         warmupIterations = 0; // Pas de warmup pour la rotation (réactivité immédiate)
//         baseSpeed = 0; // Non utilisé, le PID décide de tout
//         minSpeed = 50;  
//         maxSpeed = 90;  
//         deadZone = 3.5f;

//         // Note: pour la rotation, targetTicks ne sert qu'à vérifier grossièrement
//         // ou on garde 'value' (degrés) comme cible principale.
//         debugPrintf(DBG_MOVEMENT, "Start ROT: Val=%.1fdeg Kp=%.2f DeadZone=%.1f", value, Kp, deadZone);
//     }
// }

// void MoveTask::update(Movement &mv) {
//     if (cancelled || finished || paused) return;

//     // Timeout check
//     if (timeoutMs && (millis() - startMs) > timeoutMs) {
//         mv.stop();
//         cancelled = true;
//         finished = true;
//         return;
//     }

//     // --- Gestion du Temps (Non-bloquant) ---
//     if (millis() - lastPidLoopMs < MOVEMENT_LOOP_DELAY) {
//         return; 
//     }
//     float dt = (millis() - lastPidLoopMs) / 1000.0f; // Delta temps en secondes
//     lastPidLoopMs = millis(); 

//     // Lecture des encodeurs
//     long leftTicks, rightTicks;
//     noInterrupts();
//     leftTicks  = mv.getLeftTicks();
//     rightTicks = mv.getRightTicks();
//     interrupts();

//     // ============================================================
//     // LOGIQUE 1 : MOVE DISTANCE (On garde les roues synchronisées)
//     // ============================================================
//     if (mode == MoveTaskMode::MOVE_DISTANCE) {
        
//         long progressTicks = max(labs_long(leftTicks), labs_long(rightTicks));
        
//         // Fin ?
//         if (progressTicks >= targetTicks) {
//             mv.stop();
//             finished = true;
//             return;
//         }

//         // Erreur de synchronisation (Gauche - Droite)
//         float error = (float)(leftTicks - rightTicks);
        
//         // PID simple
//         integralError += error * Ki; // Ki est 0.0 ici par défaut
//         float correction = Kp * error + integralError;
//         float corrPWM = correction * 0.3f; 

//         // Ramp-up
//         float rampFactor = 1.0f;
//         if (loopCounter < warmupIterations) {
//             rampFactor = (float)loopCounter / (float)warmupIterations;
//             if (rampFactor < 0.05f) rampFactor = 0.05f;
//         }
//         int targetBase = minSpeed + (int)((baseSpeed - minSpeed) * rampFactor);

//         // Application moteurs
//         int leftPWM = (int)constrain((float)targetBase - corrPWM, (float)minSpeed, 255.0f);
//         int rightPWM = (int)constrain((float)targetBase + corrPWM, (float)minSpeed, 255.0f);

//         if (value >= 0.0f) {
//             mv.motorLeft->setSpeed(leftPWM); mv.motorRight->setSpeed(rightPWM);
//             mv.motorLeft->run(FORWARD);      mv.motorRight->run(FORWARD);
//         } else {
//             mv.motorLeft->setSpeed(leftPWM); mv.motorRight->setSpeed(rightPWM);
//             mv.motorLeft->run(BACKWARD);     mv.motorRight->run(BACKWARD);
//         }
//     } 
//     // ============================================================
//     // LOGIQUE 2 : ROTATION (On vise un angle précis)
//     // ============================================================
//     else { 
//         // 1. Calcul de l'angle actuel
//         long avgTicks = (labs_long(leftTicks) + labs_long(rightTicks)) / 2;
//         // On suppose que ta méthode ticksToDegrees est dispo dans mv ou via calcul
//         float currentAngle = mv.ticksToDegrees(avgTicks); 

//         // 2. Calcul de l'erreur (Cible - Actuel)
//         // 'value' contient l'angle cible absolu (ex: 90 ou -90)
//         // currentAngle est toujours positif car basé sur avgTicks absolus.
//         // On doit comparer la *valeur absolue* demandée à l'angle parcouru.
//         float targetAngleAbs = abs(value);
//         float error = targetAngleAbs - currentAngle;

//         // 3. Vérification Fin (Deadzone)
//         // Si on est assez proche (ex: erreur < 3.5°)
//         if (abs(error) <= deadZone) {
//             mv.stop();
//             finished = true;
//             debugPrintf(DBG_MOVEMENT, "Rot Done. Err=%.2f", error);
//             return;
//         }

//         // 4. PID sur l'erreur de position
//         integralError += error * dt; 
        
//         // Anti-windup basique
//         if (integralError > 50.0f) integralError = 50.0f;
//         if (integralError < -50.0f) integralError = -50.0f;

//         // Sortie PID
//         float pidOutput = (Kp * error) + (Ki * integralError);

//         // 5. Calcul Vitesse (Constrain 50 - 90 comme ta fonction)
//         int speed = (int)constrain(abs(pidOutput), (float)minSpeed, (float)maxSpeed);

//         // 6. Direction et Application
//         // Si error > 0, on n'a pas encore atteint la cible -> on continue dans le sens initial
//         // Si error < 0, on a dépassé -> on doit inverser (correction active)
        
//         bool originalDirectionIsRight = (value >= 0);
//         bool shouldGoOriginalWay = (error > 0); 
        
//         // Logique XOR : 
//         // Si on veut aller à droite (True) et qu'on doit continuer (True) -> Droite
//         // Si on veut aller à droite (True) mais qu'on a dépassé (False) -> Gauche
//         bool goRight = (originalDirectionIsRight == shouldGoOriginalWay);

//         if (goRight) {
//             mv.motorLeft->setSpeed(speed);  mv.motorRight->setSpeed(speed);
//             mv.motorLeft->run(FORWARD);     mv.motorRight->run(BACKWARD);
//         } else {
//             mv.motorLeft->setSpeed(speed);  mv.motorRight->setSpeed(speed);
//             mv.motorLeft->run(BACKWARD);    mv.motorRight->run(FORWARD);
//         }
        
//         // Debug optionnel
//         // debugPrintf(DBG_MOVEMENT, "ROT: Ang=%.1f Err=%.1f Spd=%d", currentAngle, error, speed);
//     }

//     loopCounter++;
//     mv.updateEncoderTimestamps();
// }



// // ... handleInterrupt, cancel, resume restent identiques au code précédent ...
// TaskInterruptAction MoveTask::handleInterrupt(Movement &mv, uint8_t isrFlags) {
//     if (isrFlags & ISR_FLAG_EMERGENCY) {
//         mv.stop(); cancelled = true; finished = true; return TaskInterruptAction::CANCEL;
//     }
//     if (isrFlags & ISR_FLAG_OBSTACLE) {
//         mv.stop(); paused = true; return TaskInterruptAction::PAUSE;
//     }
//     if (isrFlags & ISR_FLAG_OBSTACLE_CLEARED && paused) {
//         paused = false; lastPidLoopMs = millis(); return TaskInterruptAction::HANDLE;
//     }
//     return TaskInterruptAction::IGNORE;
// }

// void MoveTask::cancel(Movement &mv) {
//     cancelled = true; mv.stop(); finished = true;
// }

// void MoveTask::resume(Movement &mv) {
//     if (!paused || cancelled || finished) return;
//     paused = false; lastPidLoopMs = millis();
// }





// =====================================

