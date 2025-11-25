#pragma once
#include <Arduino.h>

// Simple debounced button driver for AVR
// Usage: call begin() once, then call update() regularly (e.g. every 50-200ms).
// Query isPressed(), fell(), rose(), and clearEvents().

class Button {
public:
  // pin: Arduino pin number
  // activeLow: true if pressed == LOW (INPUT_PULLUP wiring)
  // stableCounts: number of consecutive identical samples needed to change debounced state
  Button(uint8_t pin = 0, bool activeLow = true, uint8_t stableCounts = 2);

  void begin();
  // call periodically (e.g. from SensorMonitor::poll)
  void update();

  uint8_t pollFlags();

  // Debounced state
  bool isPressed() const;
  // Edge events (true only once until cleared)
  bool fell() const;
  bool rose() const;

  // Clear edge events (called by consumer after handling)
  void clearEvents();

private:
  uint8_t pin;
  bool activeLow;
  uint8_t stableCounts;

  // internals
  bool debouncedState;
  bool lastDebouncedState = false;
  bool lastRaw;
  uint8_t stableCounter;
  bool eventFell;
  bool eventRose;
};