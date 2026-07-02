#include "SenderLedButton.h"

#include <Arduino.h>

#include "WebConsoleHandler.h"
#include "config/SenderConfig.h"

namespace {
uint32_t lastErrorLedOnAt = 0;
uint32_t lastLedTestChangeAt = 0;
bool ledTestActiveState = false;
constexpr uint32_t ErrorLedPulseMs = 1000;
} // namespace

namespace SenderLedButton {

void begin() {
    pinMode(SenderConfig::LedPin1, OUTPUT);
    pinMode(SenderConfig::LedPin2, OUTPUT);
    pinMode(SenderConfig::ButtonPin, INPUT_PULLUP);
    digitalWrite(SenderConfig::LedPin1, LOW);
    digitalWrite(SenderConfig::LedPin2, LOW);
}

bool updateLedTestButton() {
    const bool pressed = digitalRead(SenderConfig::ButtonPin) == LOW;
    if (pressed == ledTestActiveState) return ledTestActiveState;

    const uint32_t now = millis();
    if (now - lastLedTestChangeAt < SenderConfig::LedTestDebounceMs) {
        return ledTestActiveState;
    }

    ledTestActiveState = pressed;
    lastLedTestChangeAt = now;
    digitalWrite(SenderConfig::LedPin1, pressed ? HIGH : LOW);
    digitalWrite(SenderConfig::LedPin2, pressed ? HIGH : LOW);
    WebConsoleHandler::log(pressed ? "[Sender] LED-Test aktiv" : "[Sender] LED-Test beendet");
    return ledTestActiveState;
}

void pulseError(uint32_t nowMs) {
    if (ledTestActiveState) return;
    digitalWrite(SenderConfig::LedPin1, HIGH);
    lastErrorLedOnAt = nowMs;
}

void update(uint32_t nowMs) {
    if (ledTestActiveState || lastErrorLedOnAt == 0) return;
    if (nowMs - lastErrorLedOnAt < ErrorLedPulseMs) return;

    digitalWrite(SenderConfig::LedPin1, LOW);
    lastErrorLedOnAt = 0;
}

bool ledTestActive() {
    return ledTestActiveState;
}

} // namespace SenderLedButton
