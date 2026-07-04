#include "SenderLedButton.h"

#include <Arduino.h>

#include "WebConsoleHandler.h"
#include "SenderLedController.h"
#include "config/DisplayConfig.h"
#include "config/SenderConfig.h"

namespace {
uint32_t lastLedTestChangeAt = 0;
bool ledTestActiveState = false;
Status::SenderLedController ledController;

void applyOutput(const Status::SenderLedOutput& output) {
    digitalWrite(SenderConfig::LedPin1, output.greenOn ? HIGH : LOW);
    digitalWrite(SenderConfig::LedPin2, output.errorOn ? HIGH : LOW);
}

void logStateChange(Status::SenderLedState previousState, Status::SenderLedState currentState) {
    if (previousState == currentState) return;
    WebConsoleHandler::log(String("[LED] state=") + Status::SenderLedController::stateName(currentState));
}
} // namespace

namespace SenderLedButton {

void begin() {
    pinMode(SenderConfig::LedPin1, OUTPUT);
    pinMode(SenderConfig::LedPin2, OUTPUT);
    pinMode(SenderConfig::ButtonPin, INPUT_PULLUP);
    ledController.reset(millis());
    applyOutput(ledController.update({millis()}));
    WebConsoleHandler::log(String("[LED] state=") + ledController.stateName());
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
    WebConsoleHandler::log(pressed ? "[LED] state=LedTest" : "[LED] LED test released");
    return ledTestActiveState;
}

void pulseError(uint32_t nowMs) {
    ledController.requestErrorPulse(nowMs);
}

void update(const Runtime::SenderLoopState& state, uint32_t lastObdResponseAt) {
    const bool canRecent = state.canRecent(state.currentMillis, DisplayConfig::CanTimeoutMs);
    const bool obdRecent = lastObdResponseAt > 0 &&
                           state.currentMillis - lastObdResponseAt <= DisplayConfig::ObdTimeoutMs;
    const bool obdTimedOut = SenderConfig::EnableOBD2 &&
                             canRecent &&
                             lastObdResponseAt > 0 &&
                             !obdRecent;

    Status::SenderLedInput input{};
    input.nowMs = state.currentMillis;
    input.senderRunning = true;
    input.canDriverReady = state.canDriverReady;
    input.canActive = state.canBusActive || canRecent;
    input.obdActive = obdRecent;
    input.espNowReady = state.espNowReady;
    input.canError = !state.canDriverReady;
    input.obdTimeout = obdTimedOut;
    input.ledTestActive = ledTestActiveState;

    const Status::SenderLedState previousState = ledController.state();
    const Status::SenderLedOutput output = ledController.update(input);
    applyOutput(output);
    logStateChange(previousState, output.state);
}

bool ledTestActive() {
    return ledTestActiveState;
}

bool vehicleOff() {
    return ledController.vehicleOff();
}

const char* stateName() {
    return ledController.stateName();
}

uint32_t lastStateChangeAt() {
    return ledController.lastStateChangeAt();
}

} // namespace SenderLedButton
