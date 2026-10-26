#include "status_led.h"

/** board status LED functions */

void LEDSetup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
}

void LEDError() {
    while (true) {
        digitalWrite(LED_PIN, HIGH);
        delay(1000);
        digitalWrite(LED_PIN, LOW);
        delay(1000);
    }
}

void LEDSuccess() {
    for (int i = 0; i < 4; i++) {
        digitalWrite(LED_PIN, LOW);
        delay(250);
        digitalWrite(LED_PIN, HIGH);
        delay(250);
    }
}

void LEDLogging() {
    digitalWrite(LED_PIN, HIGH);
}

void LEDFlying() {
    digitalWrite(LED_PIN, HIGH);
}
