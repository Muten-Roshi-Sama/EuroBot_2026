#include "Button.h"
#include "../../include/isr_flags.h"
#include "../util/Debug.h"

Button::Button(
    uint8_t pin, 
    bool activeLow, 
    uint8_t stableCounts
    ): 
    pin(pin), 
    activeLow(activeLow), 
    stableCounts(stableCounts),
    debouncedState(false), 
    lastRaw(false), 
    stableCounter(0),
    eventFell(false), 
    eventRose(false) {
        
    }

void Button::begin() {
  if (pin == 0xFF) return;
  // pinMode(pin, INPUT); // default to pullup wiring; activeLow parameter accounts for logic
  pinMode(pin, INPUT_PULLUP); // default to pullup wiring; activeLow parameter accounts for logic
  bool raw = digitalRead(pin);
  lastRaw = raw;
  debouncedState = activeLow ? (raw == LOW) : (raw == HIGH);
  stableCounter = stableCounts;
  eventFell = eventRose = false;
}

void Button::update() {
  if (pin == 0xFF) return;
  bool raw = digitalRead(pin);
  bool logical = activeLow ? (raw == LOW) : (raw == HIGH);

  if (logical == lastRaw) {
    if (stableCounter < stableCounts) stableCounter++;
  } else {
    lastRaw = logical;
    stableCounter = 1;
  }

  // Debug on state change
  static bool lastLogical = false;
  if (logical != lastLogical) {
    debugPrintf(DBG_SENSORS, "Button raw=%d logical=%d", raw, logical);
    lastLogical = logical;
  }


  if (stableCounter >= stableCounts) {
    if (debouncedState != logical) {
      if (logical) eventFell = true; // pressed
      else eventRose = true;          // released
      debouncedState = logical;
    }
    stableCounter = stableCounts;
  }
}


// continuous check
uint8_t Button::pollFlags() {
  // update debounced state
  update();

  // current debounced pressed state
  bool now = isPressed();

  // first call: initialize lastDebouncedState to current to avoid false events
  // (we rely on default init false—it's okay)
  uint8_t flags = 0;

  if (now) {
    // while held, request emergency repeatedly
    flags |= ISR_FLAG_EMERGENCY;
  } else {
    // was previously pressed and now released -> send cleared once
    if (lastDebouncedState && !now) {
      flags |= ISR_FLAG_EMERGENCY_CLEARED;
    }
  }

  // store current state for next poll
  lastDebouncedState = now;
  return flags;
}





bool Button::isPressed() const { return debouncedState; }
bool Button::fell() const { return eventFell; }
bool Button::rose() const { return eventRose; }
void Button::clearEvents() { eventFell = eventRose = false; }