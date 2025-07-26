/******************************************************************************
 * Project      : Camera Hardware Trigger Generator
 * File         : main.cpp
 * Description  : Generates precise hardware trigger pulses on a digital pin,
 *                suitable for synchronizing GigE cameras at up to 150 FPS.
 *                Supports on-demand trigger via USB serial command.
 *
 * Author       : Ammar Ajmal
 * Email        : ammarajml@gmail.com
 * Created      : 2025-07-27
 * Version      : 1.0
 * License      : (Add license info here, e.g. MIT, GPL)
 * Board        : Teensy 4.1
 ******************************************************************************/

#include <Arduino.h>

// ====== Configuration Parameters ======

// Output pin used for trigger signal (digital pin 2)
const int triggerPin = 2;

// Desired trigger interval in microseconds
// For 150 FPS, interval = 1,000,000 µs / 150 ≈ 6666 µs
const unsigned long trigger_interval_us = 6666;

// Duration of the trigger pulse (high level) in microseconds
const unsigned long pulse_width_us = 10;

// ====== Setup function: runs once ======
void setup() {
  // Initialize trigger pin as an output and set LOW initially
  pinMode(triggerPin, OUTPUT);
  digitalWrite(triggerPin, LOW);

  // Start serial communication for interactive commands and debugging
  Serial.begin(115200);

  // Wait until the serial port is open (mandatory for native USB on Teensy)
  while (!Serial) {
    // Can add timeout or additional checks here if needed
  }

  Serial.println("Camera Trigger Generator Initialized.");
  Serial.print("Trigger interval (us): ");
  Serial.println(trigger_interval_us);
  Serial.print("Pulse width (us): ");
  Serial.println(pulse_width_us);
  Serial.println("Send 't' to trigger manually.");
}

// ====== Main loop: runs repeatedly ======
void loop() {
  // Check if a command is received via USB serial
  if (Serial.available()) {
    char cmd = Serial.read();

    // Support only 't' or 'T' as valid trigger commands
    if (cmd == 't' || cmd == 'T') {
      Serial.println("Manual trigger command received.");

      // Generate a single trigger pulse
      digitalWrite(triggerPin, HIGH);
      delayMicroseconds(pulse_width_us);
      digitalWrite(triggerPin, LOW);

      Serial.println("Manual trigger pulse sent.");
    } else {
      // Unknown command received; optionally print or ignore
      Serial.print("Unknown command: ");
      Serial.println(cmd);
    }
  }

  // Generate automatic periodic trigger pulse:
  // HIGH for pulse_width_us, then LOW for (interval - pulse width)
  digitalWrite(triggerPin, HIGH);                       // Start pulse
  delayMicroseconds(pulse_width_us);                    // Pulse duration
  digitalWrite(triggerPin, LOW);                        // End pulse
  delayMicroseconds(trigger_interval_us - pulse_width_us); // Wait for remainder of interval

  // Repeat indefinitely, maintaining precise interval timing
}

