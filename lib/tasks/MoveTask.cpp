#include "MoveTask.h"
#include "settings.h"
#include "isr_flags.h"
#include "../util/Debug.h"
#include <Arduino.h> 

// Helper: absolute long pour les ticks
static inline long labs_long(long v) { return v < 0 ? -v : v; }

void MoveTask::start(Movement &mv) {
    started = true;
    startMs = millis();
    paused = false;
    finished = false;
    cancelled = false;

    mv.resetEncoders();

    integralError = 0.0f;
    loopCounter = 0;
    lastPidLoopMs = millis();

    // --- CONFIGURATION SELON LE MODE ---
    if (mode == MoveTaskMode::MOVE_DISTANCE) {
        Kp = 0.8f; 
        Ki = 0.0f;
        warmupIterations = 30;
        baseSpeed = getSpeed() ? getSpeed() : mv.defaultSpeed;
        minSpeed = 110;
        maxSpeed = 255;
        deadZone = 0.0f; 

        targetTicks = labs_long(mv.cmToTicks(value));
        
        if (value >= 0.0f) mv.forward(minSpeed); else mv.backward(minSpeed);
        debugPrintf(DBG_MOVEMENT, "Start DIST: Val=%.1fcm Ticks=%ld", value, targetTicks);

    } else {
        // --- ROTATION ---
        Kp = 1.3f; 
        Ki = 0.08f;
        warmupIterations = 0; 
        baseSpeed = 0; 
        minSpeed = 70;  
        maxSpeed = 120;  
        deadZone = 3.5f; 

        debugPrintf(DBG_MOVEMENT, "Start ROT: Val=%.1fdeg", value);
    }
}

void MoveTask::update(Movement &mv) {
    if (cancelled || finished || paused) return;

    if (timeoutMs && (millis() - startMs) > timeoutMs) {
        mv.stop(); cancelled = true; finished = true; return;
    }

    if (millis() - lastPidLoopMs < MOVEMENT_LOOP_DELAY) return; 
    float dt = (millis() - lastPidLoopMs) / 1000.0f; 
    lastPidLoopMs = millis(); 

    long leftTicks, rightTicks;
    noInterrupts();
    leftTicks  = mv.getLeftTicks();
    rightTicks = mv.getRightTicks();
    interrupts();

    // ================= DISTANCE =================
    if (mode == MoveTaskMode::MOVE_DISTANCE) {
        long progressTicks = max(labs_long(leftTicks), labs_long(rightTicks));
        
        // Fin ?
        if (progressTicks >= targetTicks) {
            mv.stop();
            finished = true;
            return;
        }

        // Erreur de synchronisation (Gauche - Droite)
        float error = (float)(leftTicks - rightTicks);
        
        // PID simple
        integralError += error * Ki; // Ki est 0.0 ici par défaut
        float correction = Kp * error + integralError;
        float corrPWM = correction * 1.0f; 

        // Ramp-up
        float rampFactor = 1.0f;
        // if (loopCounter < warmupIterations) {
        //     rampFactor = (float)loopCounter / (float)warmupIterations;
        //     if (rampFactor < 0.05f) rampFactor = 0.05f;
        // }
        int targetBase = minSpeed + (int)((baseSpeed - minSpeed) * rampFactor);

        // Application moteurs
        int leftPWM = (int)constrain((float)targetBase - corrPWM, (float)minSpeed, 255.0f);
        int rightPWM = (int)constrain((float)targetBase + corrPWM, (float)minSpeed, 255.0f);

        if (value >= 0.0f) {
            mv.motorLeft->setSpeed(leftPWM); mv.motorRight->setSpeed(rightPWM);
            mv.motorLeft->run(FORWARD);      mv.motorRight->run(FORWARD);
        } else {
            mv.motorLeft->setSpeed(leftPWM); mv.motorRight->setSpeed(rightPWM);
            mv.motorLeft->run(BACKWARD);     mv.motorRight->run(BACKWARD);
        }
        debugPrintf(DBG_MOVEMENT, "Dist: PTicks=%ld Err=%.2f Corr=%.2f L=%d R=%d distance", 
              progressTicks, error, correction, leftPWM, rightPWM, mv.getDistanceTraveled());
    } 
    
    // ================= ROTATION =================
    else { 
        long avgTicks = (labs_long(leftTicks) + labs_long(rightTicks)) / 2;
        float currentAngle = mv.ticksToDegrees(avgTicks); 

        float targetAngleAbs = fabs(value); 
        float error = targetAngleAbs - currentAngle;

        if (fabs(error) <= deadZone) {
            mv.stop(); finished = true; return;
        }

        integralError += error * dt; 
        if (integralError > 50.0f) integralError = 50.0f;
        if (integralError < -50.0f) integralError = -50.0f;

        float pidOutput = (Kp * error) + (Ki * integralError);
        int speed = (int)constrain(fabs(pidOutput), (float)minSpeed, (float)maxSpeed);

        bool goRight = ((value >= 0) == (error > 0));

        if (goRight) {
            mv.motorLeft->setSpeed(speed);  mv.motorRight->setSpeed(speed);
            mv.motorLeft->run(FORWARD);     mv.motorRight->run(BACKWARD);
        } else {
            mv.motorLeft->setSpeed(speed);  mv.motorRight->setSpeed(speed);
            mv.motorLeft->run(BACKWARD);    mv.motorRight->run(FORWARD);
        }
    }
    loopCounter++;
    mv.updateEncoderTimestamps();
}

TaskInterruptAction MoveTask::handleInterrupt(Movement &mv, uint8_t isrFlags) {
    if (isrFlags & ISR_FLAG_EMERGENCY) {
        mv.stop(); cancelled = true; finished = true; return TaskInterruptAction::CANCEL;
    }
    if (isrFlags & ISR_FLAG_OBSTACLE) {
        mv.stop(); paused = true; return TaskInterruptAction::PAUSE;
    }
    if (isrFlags & ISR_FLAG_OBSTACLE_CLEARED && paused) {
        paused = false; lastPidLoopMs = millis(); return TaskInterruptAction::HANDLE;
    }
    return TaskInterruptAction::IGNORE;
}

void MoveTask::cancel(Movement &mv) { cancelled = true; mv.stop(); finished = true; }
void MoveTask::resume(Movement &mv) { if (!paused || cancelled || finished) return; paused = false; lastPidLoopMs = millis(); }

// ====================== gyro calibration and movedistance====================
// if (subState == STATE_CALIBRATION) {
        
//         // INITIALISATION UNIQUE (Au tout premier passage)
//         if (calibration_count == 0) {
            
//             // 1. REVEIL DU MPU6050 (INDISPENSABLE !)
//             Wire.beginTransmission(0x68);
//             Wire.write(0x6B); // Registre PWR_MGMT_1
//             Wire.write(0);    // Mettre à 0 pour le réveiller
//             Wire.endTransmission(true);
            
//             // 2. CONFIGURATION SENSIBILITÉ (Optionnel mais mieux)
//             // Registre 0x1B (Gyro Config) -> 0x08 = 500dps
//             Wire.beginTransmission(0x68);
//             Wire.write(0x1B); 
//             Wire.write(0x08); 
//             Wire.endTransmission(true);

//             // Reset variables
//             gyro_z_cal_sum = 0;
//             angle_z = 0;
//             integral = 0;
//             lastError = 0;
            
//             debugPrintf(DBG_MOVEMENT, "MPU6050 Waking up...");
//             delay(50); // Petite pause pour laisser le capteur s'allumer
//         }

//         // Lecture pour calibration
//         Wire.beginTransmission(0x68);
//         Wire.write(0x47); 
//         Wire.endTransmission();
//         Wire.requestFrom(0x68, 2);
        
//         if (Wire.available() >= 2) {
//             int16_t rawZ = Wire.read() << 8 | Wire.read();
//             gyro_z_cal_sum += rawZ;
//             calibration_count++;
//         }

//         // FIN DE CALIBRATION
//         if (calibration_count >= CALIBRATION_SAMPLES) {
//             gyro_z_cal = (float)gyro_z_cal_sum / (float)CALIBRATION_SAMPLES;
//             previousTime = micros(); 
//             subState = STATE_MOVING; 
            
//             // Moteurs ON
//             int startSpeed = 150; 
//             if (value >= 0) { 
//                 mv.motorLeft->run(FORWARD); mv.motorRight->run(FORWARD);
//             } else {
//                 mv.motorLeft->run(BACKWARD); mv.motorRight->run(BACKWARD);
//             }
//             mv.motorLeft->setSpeed(startSpeed);
//             mv.motorRight->setSpeed(startSpeed);
            
//             debugPrintf(DBG_MOVEMENT, "Calibration Done. Offset: %d", (int)gyro_z_cal);
//         }
//         return; 
//     }

//     // --- SOUS-ÉTAT 2 : MOUVEMENT ---
//     if (subState == STATE_MOVING) {
        
//         long currentLeft = abs(leftTicks);
//         long currentRight = abs(rightTicks);
        
//         if (currentLeft >= targetTicks && currentRight >= targetTicks) {
//             mv.stop();
//             finished = true;
//             return;
//         }

//         unsigned long currentTime = micros();
//         float dt = (currentTime - previousTime) / 1000000.0;
//         previousTime = currentTime;
//         if (dt > 0.1 || dt <= 0.0) dt = 0.001; 

//         // LECTURE GYRO
//         int16_t raw_gyro_z = 0; // On stocke la valeur brute pour l'afficher
//         Wire.beginTransmission(0x68);
//         Wire.write(0x47);
//         Wire.endTransmission();
//         Wire.requestFrom(0x68, 2);
        
//         if (Wire.available() >= 2) {
//             raw_gyro_z = Wire.read() << 8 | Wire.read();
            
//             // Division par 65.5 car on a configuré 500dps plus haut
//             float gyro_z_dps = (raw_gyro_z - gyro_z_cal) / 65.5;
            
//             // J'ai enlevé la deadzone pour le test, on veut tout voir !
//             angle_z += gyro_z_dps * dt;
//         }

//         // PID
//         float error = 0 - angle_z;
//         integral += error * dt;
//         if (integral > 200) integral = 200;
//         if (integral < -200) integral = -200;
//         float derivative = (error - lastError) / dt;
//         lastError = error;
//         float outputCorrection = (Kp_gyro * error) + (Ki_gyro * integral) + (Kd_gyro * derivative);

//         // MOTEURS
//         int targetBase = DEFAULT_SPEED; 
//         int leftSpeed = constrain(targetBase - (int)outputCorrection, 0, 255);
//         int rightSpeed = constrain(targetBase + (int)outputCorrection, 0, 255);

//         mv.motorLeft->setSpeed(leftSpeed);
//         mv.motorRight->setSpeed(rightSpeed);

//         // DEBUG CRITIQUE
//         static long lastPrint = 0;
//         if (millis() - lastPrint > 100) {
//             // J'affiche RAW pour voir si le capteur réagit
//             debugPrintf(DBG_MOVEMENT, "RAW:%d A:%d L:%d R:%d", 
//                 raw_gyro_z,           // VALEUR BRUTE (Si elle ne bouge pas, capteur HS/Mal branché)
//                 (int)(angle_z ),  // Angle x10
//                 leftSpeed, rightSpeed);
//             lastPrint = millis();
//         }
//     }