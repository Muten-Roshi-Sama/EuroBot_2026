// Simple I2C probe + PCF8574/ADXL345 test harness
// This file is intended for quick hardware verification. Debug prints
// have been removed for cleanliness; re-enable logging by restoring
// the Serial.print lines or by using the project's Debug API.

#include <Arduino.h>
#include <Wire.h>
#include "I2CDevices.h"

// Address ranges to probe
const uint8_t PCF8574_RANGES[][2] = {{0x20, 0x27}, {0x38, 0x3F}};
const uint8_t ADXL_ADDRS[] = {0x53, 0x1D};

PCF8574 pcf;
ADXL345 accel;

int found_pcf_addr = -1;
int found_adxl_addr = -1;

// Debounce state for PCF8574 port readings
uint8_t last_port = 0xFF;
uint8_t stable_port = 0xFF;
unsigned long last_change_time = 0;
const unsigned long debounce_ms = 50;

// Configuration: set true if the switch is active-high (pressed when pin is HIGH)
// If your switch pulls the pin to GND when pressed, leave as false (active-low).
const bool PCF_P0_ACTIVE_HIGH = false; // change to true if your wiring makes P0 HIGH when pressed
const bool PCF_P1_ACTIVE_HIGH = false; // change to true if your magnetic switch is active-high
// LED on P5 configuration: set true if LED turns ON when pin is HIGH
const bool PCF_P5_ACTIVE_HIGH = false; // set accordingly to your LED wiring

// LED blink parameters
const unsigned long led_blink_ms = 500;
unsigned long last_led_toggle = 0;
bool led_state = false; // false = off, true = on (interpreted according to PCF_P5_ACTIVE_HIGH)
uint8_t pcf_output_port = 0xFF; // value last written to PCF8574 (start high)

// scanBus: optional helper to print all addresses responding on I2C.
// Disabled by default (commented prints) — useful during manual debugging.
void scanBus() {
    Serial.println("I2C bus scan:");
    for (uint8_t addr = 1; addr < 127; ++addr) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf(" - 0x%02X\n", addr);
            delay(2);
        }
    }
}

// tryFindPCF: probe common PCF8574 address ranges and store first found
// address in `found_pcf_addr`. Prints are disabled; restore if needed.
void tryFindPCF() {
    for (size_t r = 0; r < sizeof(PCF8574_RANGES)/sizeof(PCF8574_RANGES[0]); ++r) {
        for (uint8_t a = PCF8574_RANGES[r][0]; a <= PCF8574_RANGES[r][1]; ++a) {
            if (pcf.begin(a)) {
                found_pcf_addr = a;
                Serial.printf("Found PCF8574 at 0x%02X\n", a);
                return;
            }
        }
    }
    Serial.println("No PCF8574 detected in standard ranges");
}

// tryFindADXL: detect ADXL345 on common addresses and save the address.
void tryFindADXL() {
    for (size_t i = 0; i < sizeof(ADXL_ADDRS); ++i) {
        if (accel.begin(ADXL_ADDRS[i])) {
            found_adxl_addr = ADXL_ADDRS[i];
            Serial.printf("Found ADXL345 at 0x%02X\n", found_adxl_addr);
            return;
        }
    }
    Serial.println("No ADXL345 detected at 0x53/0x1D");
}

void setup() {
    Serial.begin(115200); // enable for interactive debugging
    delay(100);

    // Initialize I2C bus
    Wire.begin();
    delay(50);

    // Optional probes (scan, detect devices)
    scanBus();
    tryFindPCF();
    tryFindADXL();

    Serial.println("Setup done. Reading loop starting...");
}

void loop() {
    // Priority: ADXL then PCF8574 switch on P0
    // If ADXL present, read acceleration values. Prints are omitted by
    // default; re-enable Serial prints for verbose monitoring.
    if (found_adxl_addr >= 0) {
        int16_t ax, ay, az;
        if (accel.readAccel(ax, ay, az)) {
            Serial.printf("ADXL: ax=%6d ay=%6d az=%6d\n", ax, ay, az);
        } else {
            Serial.println("ADXL: read failed");
        }
    } else {
        Serial.println("ADXL: not present");
    }

    if (found_pcf_addr >= 0) {
        uint8_t port = 0xFF;
        if (pcf.read(port)) {
            // Debounce logic: update last_port and stable_port only after stable for debounce_ms
            unsigned long now = millis();
            if (port != last_port) {
                last_change_time = now;
                last_port = port;
            } else {
                if ((now - last_change_time) > debounce_ms && port != stable_port) {
                    stable_port = port;
                }
            }

            // Compute pressed/released with configurable active-high/low behaviour.
            bool raw_p0 = (stable_port & 0x01) != 0; // true if pin reads HIGH
            bool raw_p1 = (stable_port & 0x02) != 0; // true if pin reads HIGH
            bool switch_p0 = PCF_P0_ACTIVE_HIGH ? raw_p0 : !raw_p0;
            bool switch_p1 = PCF_P1_ACTIVE_HIGH ? raw_p1 : !raw_p1;
            // Build a human-readable binary string for stable_port (8 bits)
            char bits[9];
            for (int i = 0; i < 8; ++i) bits[7 - i] = (stable_port & (1 << i)) ? '1' : '0';
            bits[8] = '\0';
            // Uncomment the following to print port & button states:
            Serial.printf("PCF8574(0x%02X) port=0x%02X raw=0b%s P0=%s P1=%s\n",
                           found_pcf_addr, stable_port, bits,
                          switch_p0 ? "PRESSED" : "released",
                           switch_p1 ? "PRESSED" : "released");

            // --- Blink LED on P5 to test PCF8574 outputs ---
            now = millis();
            if ((now - last_led_toggle) >= led_blink_ms) {
                last_led_toggle = now;
                led_state = !led_state;
                // compute desired output bit for P5 according to active-high config
                bool p5_output_level = PCF_P5_ACTIVE_HIGH ? led_state : !led_state;
                if (p5_output_level) {
                    pcf_output_port |= (1 << 5); // set bit to HIGH (releases pin)
                } else {
                    pcf_output_port &= ~(1 << 5); // clear bit to drive LOW
                }
                if (!pcf.write(pcf_output_port)) {
                    // Serial.println("PCF8574: write failed when toggling P5");
                } else {
                    // Serial.printf("PCF8574: wrote 0x%02X to set P5 %s\n", pcf_output_port, led_state ? "ON" : "OFF");
                }
            }
        } else {
            Serial.println("PCF8574: read failed");
        }
    } else {
        Serial.println("PCF8574: not present");
    }

    delay(250);
}
