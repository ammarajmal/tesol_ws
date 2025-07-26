/******************************************************************************
 * Project      : Camera Hardware Trigger Generator with LED Feedback
 * File         : main.cpp
 * Description  : Teensy 4.1 – Periodic trigger, pushbutton & serial/manual trigger,
 *                with clear LED indication and serial prints for manual triggers.
 * Author       : Ammar Ajmal
 * Email        : ammarajml@gmail.com
 * Version      : 1.3  (2025-07-27)
 * Board        : Teensy 4.1
 ******************************************************************************/


#include <Arduino.h>

const int triggerPin = 2;   // Output trigger pin (to camera)
const int ledPin = 13;      // Teensy onboard LED
const int buttonPin = 3;    // Pushbutton between pin 3 and GND

const unsigned long trigger_interval_us = 6666; // 150Hz -> interval
const unsigned long pulse_width_us = 10;        // 10us trigger pulse
const unsigned long debounce_ms = 50;           // Button debounce interval

// State-tracking for button debounce
bool lastButtonPhysical = HIGH;
unsigned long lastDebounceTime = 0;

// Function to fire both trigger and LED for N ms, and print a message
void fireTriggerWithLED(const char* src, unsigned long ledOn_ms) {
    Serial.print("Trigger via ");
    Serial.println(src);
    digitalWrite(triggerPin, HIGH);
    digitalWrite(ledPin, HIGH);
    delayMicroseconds(pulse_width_us);
    digitalWrite(triggerPin, LOW);
    delay(ledOn_ms);
    digitalWrite(ledPin, LOW);
}

void setup() {
    pinMode(triggerPin, OUTPUT);
    digitalWrite(triggerPin, LOW);
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW);
    pinMode(buttonPin, INPUT_PULLUP);

    Serial.begin(115200);
    while (!Serial);
    Serial.println("Teensy Camera Trigger & Button/LED Feedback (v1.3)");
    Serial.println("Periodic (150Hz), pushbutton or serial ('t') triggers LED and output pulse.");
}

void loop() {
    // --- Button debounce and event detection ---
    bool buttonPhysical = digitalRead(buttonPin); // LOW = pressed, HIGH = not pressed

    // Debounce logic: only react to new presses
    if (buttonPhysical != lastButtonPhysical) {
        lastDebounceTime = millis();
    }

    if (lastButtonPhysical == HIGH && buttonPhysical == LOW) { // just pressed!
        if ((millis() - lastDebounceTime) > debounce_ms) {
            // Fire trigger and LED for 1 second, print message
            fireTriggerWithLED("push button", 1000); // 1s LED ON
            delay(200); // Simple long-press guard
        }
    }
    lastButtonPhysical = buttonPhysical;

    // --- Serial trigger ('t' or 'T') fires LED and output ---
    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 't' || cmd == 'T') {
            fireTriggerWithLED("serial", 1000);
        } else {
            Serial.print("Unknown command: "); Serial.println(cmd);
        }
    }

    // --- Periodic (automatic) trigger -- no LED feedback, just output ---
    static unsigned long nextTrigger = 0;
    unsigned long now = micros();
    if (now >= nextTrigger) {
        digitalWrite(triggerPin, HIGH);
        delayMicroseconds(pulse_width_us);
        digitalWrite(triggerPin, LOW);
        nextTrigger = now + trigger_interval_us;
    }
}
