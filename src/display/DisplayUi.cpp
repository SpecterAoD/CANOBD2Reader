// Display rendering (LilyGO T-Display S3, 320x170, ST7789 8-bit parallel).
//
// Flicker-free by design: every frame is drawn into a full-screen sprite in
// RAM (PSRAM if available) and pushed to the panel in one transfer. Nothing is
// ever cleared on the visible screen. Values that miss an update are greyed
// out instead of being replaced by placeholders (see LiveValueView).
//
// Buttons: BOOT (GPIO0) short = next page, GPIO14 short = previous page,
// either long = main page, BOOT very long = toggle diagnostic pages.
// Pages:
//   0 Fahren  1 Motor  2 Verbrauch  3 Verbindung
//   4 DTC/UDS 5 CAN-Sniffer (dev builds) 6 Weitere Werte 7 Power
#include "DisplayUi.h"
#include "DisplayRenderer.h"

#include <TFT_eSPI.h>

#include "DiagnosticLog.h"
#include "DisplayData.h"
#include "ButtonGesture.h"
#include "driver/ledc.h"

namespace DisplayCanvas {
TFT_eSPI tft;
TFT_eSprite frame(&tft);
}

namespace {

bool frameReady = false;

using namespace DisplayCanvas;

uint32_t lastRenderMs = 0;
bool sleeping = false;
DisplayLogic::ButtonGesture nextButton(DisplayConfig::ButtonDebounceMs, DisplayConfig::LongPressMs,
                                       DisplayConfig::VeryLongPressMs);
DisplayLogic::ButtonGesture previousButton(DisplayConfig::ButtonDebounceMs, DisplayConfig::LongPressMs,
                                           DisplayConfig::VeryLongPressMs);

// ---------------------------------------------------------------------------
// Panel bring-up
// ---------------------------------------------------------------------------

struct InitCommand {
  uint8_t command;
  uint8_t data[14];
  uint8_t length;  // bit7: wait 120 ms afterwards
};

// Panel revision specific ST7789 init (LilyGO reference sequence).
constexpr InitCommand PanelInit[] = {
    {0x11, {0}, 0x80},
    {0x3A, {0x05}, 1},
    {0xB2, {0x0B, 0x0B, 0x00, 0x33, 0x33}, 5},
    {0xB7, {0x75}, 1},
    {0xBB, {0x28}, 1},
    {0xC0, {0x2C}, 1},
    {0xC2, {0x01}, 1},
    {0xC3, {0x1F}, 1},
    {0xC6, {0x13}, 1},
    {0xD0, {0xA7}, 1},
    {0xD0, {0xA4, 0xA1}, 2},
    {0xD6, {0xA1}, 1},
    {0xE0, {0xF0, 0x05, 0x0A, 0x06, 0x06, 0x03, 0x2B, 0x32, 0x43, 0x36, 0x11, 0x10, 0x2B, 0x32}, 14},
    {0xE1, {0xF0, 0x08, 0x0C, 0x0B, 0x09, 0x24, 0x2B, 0x22, 0x43, 0x38, 0x15, 0x16, 0x2F, 0x37}, 14},
};

void applyPanelInit() {
  for (const InitCommand& entry : PanelInit) {
    tft.writecommand(entry.command);
    const unsigned count = entry.length & 0x7Fu;
    for (unsigned index = 0; index < count; ++index) tft.writedata(entry.data[index]);
    if (entry.length & 0x80) delay(120);
  }
}

void setBacklight(uint8_t duty) {
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void setupBacklight() {
  ledc_timer_config_t timer = {};
  timer.speed_mode = LEDC_LOW_SPEED_MODE;
  timer.duty_resolution = LEDC_TIMER_8_BIT;
  timer.timer_num = LEDC_TIMER_0;
  timer.freq_hz = 10000;
  timer.clk_cfg = LEDC_AUTO_CLK;
  ledc_timer_config(&timer);

  ledc_channel_config_t channel = {};
  channel.gpio_num = DisplayConfig::BacklightPin;
  channel.speed_mode = LEDC_LOW_SPEED_MODE;
  channel.channel = LEDC_CHANNEL_0;
  channel.timer_sel = LEDC_TIMER_0;
  channel.duty = 0;
  ledc_channel_config(&channel);
  setBacklight(DisplayConfig::BacklightOn);
}

void createFrameBuffer() {
  // 16 bit: 108 KB, normally in PSRAM. Fall back to 8 bit (54 KB) if needed.
  frame.setColorDepth(16);
  frameReady = frame.createSprite(Width, Height) != nullptr;
  if (!frameReady) {
    frame.setColorDepth(8);
    frameReady = frame.createSprite(Width, Height) != nullptr;
  }
  DiagnosticLog::appendf("[DISPLAY] Framebuffer %s (%d bit)", frameReady ? "ok" : "FEHLT", frame.getColorDepth());
}

void showBootScreen() {
  tft.fillScreen(DisplayConfig::Background);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(DisplayConfig::Accent, DisplayConfig::Background);
  tft.drawString("CAN OBD2", Width / 2, Height / 2 - 14, 4);
  tft.setTextColor(DisplayConfig::Muted, DisplayConfig::Background);
  tft.drawString(String("Display ") + ProjectConfig::FirmwareVersion, Width / 2, Height / 2 + 18, 2);
}

} // namespace

namespace DisplayUi {

void begin() {
  pinMode(DisplayConfig::NextPageButtonPin, INPUT_PULLUP);
  pinMode(DisplayConfig::PreviousPageButtonPin, INPUT_PULLUP);
  if (DisplayConfig::PowerPin >= 0) {
    pinMode(DisplayConfig::PowerPin, OUTPUT);
    digitalWrite(DisplayConfig::PowerPin, DisplayConfig::PowerOnLevel);
    delay(DisplayConfig::PowerStabilizeMs);
  }
  tft.init();
  applyPanelInit();
  tft.setRotation(DisplayConfig::Rotation);
  setupBacklight();
  showBootScreen();
  createFrameBuffer();
}

bool ready() { return frameReady; }

void handleButton() {
  using namespace DisplayData;
  const uint32_t now = millis();
  const uint8_t pages = visiblePageCount();
  const auto next = nextButton.update(digitalRead(DisplayConfig::NextPageButtonPin) == LOW, now);
  const auto previous = previousButton.update(digitalRead(DisplayConfig::PreviousPageButtonPin) == LOW, now);
  if (next == DisplayLogic::ButtonEvent::None && previous == DisplayLogic::ButtonEvent::None) return;

  runtime().displaySleepRequested = false;  // any press wakes the display
  if (next == DisplayLogic::ButtonEvent::VeryLong) {
    setDiagnosticPagesEnabled(!runtime().diagnosticPagesEnabled);
    currentPage = DisplayConfig::MainPageIndex;
  } else if (next == DisplayLogic::ButtonEvent::Long || previous == DisplayLogic::ButtonEvent::Long) {
    currentPage = DisplayConfig::MainPageIndex;
  } else if (next == DisplayLogic::ButtonEvent::Short) {
    currentPage = static_cast<uint8_t>((currentPage + 1) % pages);
  } else if (previous == DisplayLogic::ButtonEvent::Short) {
    currentPage = static_cast<uint8_t>((currentPage + pages - 1) % pages);
  }
  lastRenderMs = 0;  // render the new page immediately
}

void renderIfDue() {
  using namespace DisplayData;
  if (runtime().displaySleepRequested) {
    if (!sleeping) {
      setBacklight(DisplayConfig::BacklightSleep);
      sleeping = true;
      DiagnosticLog::appendf("[DISPLAY] Schlafmodus");
    }
    return;
  }
  if (sleeping) {
    sleeping = false;
    setBacklight(DisplayConfig::BacklightOn);
    lastRenderMs = 0;
  }

  const uint32_t now = millis();
  if (!frameReady || (lastRenderMs != 0 && now - lastRenderMs < DisplayConfig::RenderIntervalMs)) return;
  lastRenderMs = now;
  DisplayRenderer::render();
}

} // namespace DisplayUi
