#include "MoveTask.h"
#include "settings.h"
#include "isr_flags.h"

// Libs
#include "../util/Debug.h"


// debugPrintf(DBG_MOVEMENT, "", );


// Helper: absolute long
static inline long labs_long(long v) { return v < 0 ? -v : v; }
int MoveTask::baseSpeed = 0;
int MoveTask::loopCounter = 0;
int MoveTask::warmupIterations = 30;
float MoveTask::Kp = 0.8f;
float MoveTask::Ki = 0.0f;
float MoveTask::integralError = 0.0f;
int MoveTask::minSpeed = 80;



void MoveTask::start(Movement &mv) {
    started = true;
    startMs = millis();
    paused = false;
    finished = false;
    cancelled = false;

  // initialize controller / runtime state
  mv.resetEncoders();

  baseSpeed = getSpeed() ? getSpeed() : mv.defaultSpeed;
  loopCounter = 0;
  warmupIterations = 30; // ~3s at 100ms tick (adjustable)
  Kp = 0.8f;
  Ki = 0.0f;
  integralError = 0.0f;
  minSpeed = 110; // safety minimum PWM to avoid stalling

  // Log task start (short, single line)
  int value_x10 = (int)(value * 10.0f + 0.5f);
  debugPrintf(DBG_MOVEMENT, "MoveTask::start %s val=%d/10cm tgt=%ld bs=%d",
    (mode == MoveTaskMode::MOVE_DISTANCE) ? "DIST" : "ROT",
    value_x10, (long)0, baseSpeed); // targetTicks updated below

  // Log motor pointers so we can confirm Movement initialized properly
  if (mv.motorLeft == nullptr || mv.motorRight == nullptr) {
    debugPrintf(DBG_MOVEMENT, "MoveTask::start WARNING motors null L=%p R=%p", (void*)mv.motorLeft, (void*)mv.motorRight);
  } else {
    debugPrintf(DBG_MOVEMENT, "MoveTask::start motors L=%p R=%p minSpeed=%d", (void*)mv.motorLeft, (void*)mv.motorRight, minSpeed);
  }


  // MOVE
  if (mode == MoveTaskMode::MOVE_DISTANCE) {
    targetTicks = labs_long(mv.cmToTicks(value));
    debugPrintf(DBG_MOVEMENT, "MoveTask targetTicks=%ld (%.1f cm)", targetTicks, value);
    // start gently
    if (value >= 0.0f) {
      mv.forward(minSpeed);
    } else {
      mv.backward(minSpeed);
    }

    // ROTATE_ANGLE
  } else if (mode == MoveTaskMode::ROTATE_ANGLE) { 
    targetTicks = labs_long(mv.degreesToTicks(value));
    // for rotation, we start both motors at minSpeed in correct directions
    if (value >= 0.0f) {
      mv.rotateRight(minSpeed);
    } else {
      mv.rotateLeft(minSpeed);
    }
  }



}

void MoveTask::update(Movement &mv) {
    if (cancelled || finished || paused) return;

  // Read encoder ticks atomically
  long leftTicks, rightTicks;
  noInterrupts();
  leftTicks  = mv.getLeftTicks();
  rightTicks = mv.getRightTicks();
  interrupts();

  // Progress measurement
  long progressTicks;
  if (mode == MoveTaskMode::MOVE_DISTANCE) {
    progressTicks = max(labs_long(leftTicks), labs_long(rightTicks));
  } else { // rotation
    progressTicks = (labs_long(leftTicks) + labs_long(rightTicks)) / 2;
  }

  // Check finish condition
  if (progressTicks >= targetTicks) {
    debugPrintf(DBG_MOVEMENT, "MoveTask progress : prog=%ld / tgt=%ld", progressTicks, targetTicks);
    mv.stop();
    finished = true;
    return;
  }

  // Timeout handling
  if (timeoutMs && (millis() - startMs) > timeoutMs) {
    mv.stop();
    cancelled = true;
    finished = true;
    return;
  }

  // Controller: correct left/right imbalance using proportional term
  // error = left - right (positive -> left ahead -> slow left / speed up right)
  float error = (float)(leftTicks - rightTicks);
  // integral (very small or disabled)
  integralError += error * (Ki);
  // anti-windup clamp
  if (integralError > 1000) integralError = 1000;
  if (integralError < -1000) integralError = -1000;

  float correction = Kp * error + integralError;
  // Convert correction (ticks) to PWM delta (heuristic)
  // The factor below maps ticks-difference into PWM difference; tune as needed.
  float corrPWM = correction * 0.3f;

  // Warmup ramp: gradually increase target base PWM from minSpeed to baseSpeed
  float rampFactor = 1.0f;
  if (loopCounter < warmupIterations) {
    rampFactor = (float)loopCounter / (float)warmupIterations;
    if (rampFactor < 0.05f) rampFactor = 0.05f;
  }

  int targetBase = minSpeed + (int)((baseSpeed - minSpeed) * rampFactor);

  // Compute left/right speed depending on motion mode
  int leftPWM = targetBase;
  int rightPWM = targetBase;

  if (mode == MoveTaskMode::MOVE_DISTANCE) {
    leftPWM = (int)constrain((float)targetBase - corrPWM, (float)minSpeed, 255.0f);
    rightPWM = (int)constrain((float)targetBase + corrPWM, (float)minSpeed, 255.0f);
    // apply speeds preserving direction (forward/backward determined by start)
    // we assume previously set directions via mv.forward()/mv.backward()
    if (value >= 0.0f) {
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
  } else { // ROTATE_ANGLE
    // For rotation, one wheel forward, one backward. Correction sign flips accordingly.
    // leftPWM and rightPWM are magnitudes; directions depend on desired rotation sign.
    float corrRot = corrPWM;
    int magLeft = (int)constrain((float)targetBase + corrRot, (float)minSpeed, 255.0f);
    int magRight = (int)constrain((float)targetBase - corrRot, (float)minSpeed, 255.0f);
    if (value >= 0.0f) {
      // rotate right: left forward, right backward
      mv.motorLeft->setSpeed(magLeft);
      mv.motorRight->setSpeed(magRight);
      mv.motorLeft->run(FORWARD);
      mv.motorRight->run(BACKWARD);
    } else {
      // rotate left: left backward, right forward
      mv.motorLeft->setSpeed(magLeft);
      mv.motorRight->setSpeed(magRight);
      mv.motorLeft->run(BACKWARD);
      mv.motorRight->run(FORWARD);
    }
  }

  loopCounter++;
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
  debugPrintf(DBG_MOVEMENT, "MoveTask cancel mode=%s",(mode == MoveTaskMode::MOVE_DISTANCE) ? "DIST" : "ROT");
  cancelled = true;
  mv.stop();
  finished = true;
}

void MoveTask::resume(Movement &mv) {
  if (!paused || cancelled || finished) return;
  paused = false;
  uint8_t s = getSpeed() ? getSpeed() : mv.defaultSpeed;
  if (mode == MoveTaskMode::MOVE_DISTANCE) {
    if (value >= 0.0f) mv.forward(s); else mv.backward(s);
  } else {
    if (value >= 0.0f) mv.rotateRight(s); else mv.rotateLeft(s);
  }
  debugPrintf(DBG_MOVEMENT, "MoveTask resume mode=%s bs=%d", (mode == MoveTaskMode::MOVE_DISTANCE) ? "DIST" : "ROT", s);
}



