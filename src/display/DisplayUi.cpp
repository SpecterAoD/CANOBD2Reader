#include "DisplayUi.h"
#include "DisplayData.h"
#include "DiagnosticLog.h"
#include "driver/ledc.h"
#include <cstring>

namespace {
  TFT_eSPI tft = TFT_eSPI();
  bool lastButtonPressed = false;
  uint32_t buttonPressedAt = 0;
  bool longPressHandled = false;
  uint32_t lastUiStatsLogMs = 0;
  bool fullPageRedraw = true;
  bool displaySleeping = false;
  bool lastEspNowConnected = false;

  struct DisplayInitCommand {
    uint8_t command;
    uint8_t data[14];
    uint8_t length;
  };

  constexpr DisplayInitCommand kDisplayInitSequence[] = {
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

  void logUi(const char* message) {
    DiagnosticLog::appendf("[display-ui] %s", message == nullptr ? "" : message);
  }

  void logPinState(const char* label, int pin) {
    DiagnosticLog::appendf("[display-ui] %s pin %d state=%d",
                           label == nullptr ? "" : label,
                           pin,
                           digitalRead(pin));
  }

  void applyPanelRevisionInit() {
    logUi("applying panel revision init");
    for (const DisplayInitCommand& entry : kDisplayInitSequence) {
      tft.writecommand(entry.command);
      const uint8_t dataLength = static_cast<uint8_t>(entry.length & 0x7F);
      for (uint8_t index = 0; index < dataLength; ++index) {
        tft.writedata(entry.data[index]);
      }
      if (entry.length & 0x80) {
        delay(120);
      }
    }
  }

  void setupDisplayPower() {
    if (DisplayConfig::PowerPin < 0) return;

    logUi("enabling display power");
    pinMode(DisplayConfig::PowerPin, OUTPUT);
    digitalWrite(DisplayConfig::PowerPin, DisplayConfig::PowerOnLevel);
    delay(DisplayConfig::PowerStabilizeMs);
    logPinState("power", DisplayConfig::PowerPin);
  }

  void setBacklight(uint8_t brightness) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, brightness);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
  }

  void setupBacklight() {
    logUi("configuring backlight pwm");
    pinMode(DisplayConfig::BacklightPin, OUTPUT);
    digitalWrite(DisplayConfig::BacklightPin, DisplayConfig::BacklightOn > 0 ? HIGH : LOW);

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
    channel.intr_type = LEDC_INTR_DISABLE;
    channel.timer_sel = LEDC_TIMER_0;
    channel.duty = 0;
    ledc_channel_config(&channel);

    setBacklight(DisplayConfig::BacklightOn);
    // Fallback: some panel revisions ignore PWM until pin is explicitly driven.
    digitalWrite(DisplayConfig::BacklightPin, DisplayConfig::BacklightOn > 0 ? HIGH : LOW);
    logUi("backlight enabled");
  }

  void drawStatusBar() {
    using namespace DisplayData;
    const bool connected = isEspNowConnected();
    if (lastEspNowConnected && !connected) {
      DiagnosticLog::appendf("[DISPLAY] ESP-NOW timeout");
    }
    lastEspNowConnected = connected;

    const uint16_t barColor = connected ? DisplayConfig::Panel : DisplayConfig::Error;
    tft.fillRect(0, 0, tft.width(), 20, barColor);
    tft.setTextSize(1);
    tft.setTextDatum(ML_DATUM);
    tft.setTextColor(DisplayConfig::Text, barColor);
    tft.drawString(connected ? "ESP-NOW OK" : "VERBINDUNG VERLOREN", 4, 10);

    tft.setTextDatum(MR_DATUM);
    const uint8_t visiblePages = runtime().diagnosticPagesEnabled ? DisplayConfig::PageCount : DisplayConfig::NormalPageCount;
    String right = "S" + String(currentPage + 1) + "/" + String(visiblePages);
    if (runtime().diagnosticPagesEnabled) right += " DIAG";
    if (DisplayConfig::EnableStartupValueOverlay && millis() < DisplayConfig::StartupValueOverlayMs) {
      right += " DIAG";
    }
    if (runtime().lastReceivedAt > 0) right += "  " + String((millis() - runtime().lastReceivedAt) / 1000) + "s";
    tft.drawString(right, tft.width() - 4, 10);
  }

  bool startupOverlayActive() {
    return DisplayConfig::EnableStartupValueOverlay && millis() < DisplayConfig::StartupValueOverlayMs;
  }

  const uint8_t* glyph5x7(char c) {
    static const uint8_t gSpace[7] = {0,0,0,0,0,0,0};
    static const uint8_t gDot[7] = {0,0,0,0,0,0b01100,0b01100};
    static const uint8_t gSlash[7] = {0b00001,0b00010,0b00100,0b01000,0b10000,0,0};
    static const uint8_t gDash[7] = {0,0,0,0b11111,0,0,0};
    static const uint8_t gPercent[7] = {0b11001,0b11010,0b00100,0b01000,0b10110,0b00110,0};
    static const uint8_t gColon[7] = {0,0b01100,0b01100,0,0b01100,0b01100,0};

    static const uint8_t g0[7] = {0b01110,0b10001,0b10011,0b10101,0b11001,0b10001,0b01110};
    static const uint8_t g1[7] = {0b00100,0b01100,0b00100,0b00100,0b00100,0b00100,0b01110};
    static const uint8_t g2[7] = {0b01110,0b10001,0b00001,0b00010,0b00100,0b01000,0b11111};
    static const uint8_t g3[7] = {0b11110,0b00001,0b00001,0b00110,0b00001,0b00001,0b11110};
    static const uint8_t g4[7] = {0b00010,0b00110,0b01010,0b10010,0b11111,0b00010,0b00010};
    static const uint8_t g5[7] = {0b11111,0b10000,0b11110,0b00001,0b00001,0b10001,0b01110};
    static const uint8_t g6[7] = {0b00110,0b01000,0b10000,0b11110,0b10001,0b10001,0b01110};
    static const uint8_t g7[7] = {0b11111,0b00001,0b00010,0b00100,0b01000,0b01000,0b01000};
    static const uint8_t g8[7] = {0b01110,0b10001,0b10001,0b01110,0b10001,0b10001,0b01110};
    static const uint8_t g9[7] = {0b01110,0b10001,0b10001,0b01111,0b00001,0b00010,0b11100};

    static const uint8_t gA[7] = {0b01110,0b10001,0b10001,0b11111,0b10001,0b10001,0b10001};
    static const uint8_t gB[7] = {0b11110,0b10001,0b10001,0b11110,0b10001,0b10001,0b11110};
    static const uint8_t gC[7] = {0b01110,0b10001,0b10000,0b10000,0b10000,0b10001,0b01110};
    static const uint8_t gD[7] = {0b11100,0b10010,0b10001,0b10001,0b10001,0b10010,0b11100};
    static const uint8_t gE[7] = {0b11111,0b10000,0b10000,0b11110,0b10000,0b10000,0b11111};
    static const uint8_t gF[7] = {0b11111,0b10000,0b10000,0b11110,0b10000,0b10000,0b10000};
    static const uint8_t gG[7] = {0b01110,0b10001,0b10000,0b10111,0b10001,0b10001,0b01111};
    static const uint8_t gH[7] = {0b10001,0b10001,0b10001,0b11111,0b10001,0b10001,0b10001};
    static const uint8_t gI[7] = {0b01110,0b00100,0b00100,0b00100,0b00100,0b00100,0b01110};
    static const uint8_t gK[7] = {0b10001,0b10010,0b10100,0b11000,0b10100,0b10010,0b10001};
    static const uint8_t gL[7] = {0b10000,0b10000,0b10000,0b10000,0b10000,0b10000,0b11111};
    static const uint8_t gM[7] = {0b10001,0b11011,0b10101,0b10101,0b10001,0b10001,0b10001};
    static const uint8_t gN[7] = {0b10001,0b11001,0b10101,0b10011,0b10001,0b10001,0b10001};
    static const uint8_t gO[7] = {0b01110,0b10001,0b10001,0b10001,0b10001,0b10001,0b01110};
    static const uint8_t gP[7] = {0b11110,0b10001,0b10001,0b11110,0b10000,0b10000,0b10000};
    static const uint8_t gQ[7] = {0b01110,0b10001,0b10001,0b10001,0b10101,0b10010,0b01101};
    static const uint8_t gR[7] = {0b11110,0b10001,0b10001,0b11110,0b10100,0b10010,0b10001};
    static const uint8_t gS[7] = {0b01111,0b10000,0b10000,0b01110,0b00001,0b00001,0b11110};
    static const uint8_t gT[7] = {0b11111,0b00100,0b00100,0b00100,0b00100,0b00100,0b00100};
    static const uint8_t gU[7] = {0b10001,0b10001,0b10001,0b10001,0b10001,0b10001,0b01110};
    static const uint8_t gV[7] = {0b10001,0b10001,0b10001,0b10001,0b10001,0b01010,0b00100};
    static const uint8_t gW[7] = {0b10001,0b10001,0b10001,0b10101,0b10101,0b10101,0b01010};
    static const uint8_t gX[7] = {0b10001,0b01010,0b00100,0b00100,0b00100,0b01010,0b10001};
    static const uint8_t gY[7] = {0b10001,0b10001,0b01010,0b00100,0b00100,0b00100,0b00100};
    static const uint8_t gZ[7] = {0b11111,0b00001,0b00010,0b00100,0b01000,0b10000,0b11111};

    switch (c) {
      case ' ': return gSpace;
      case '.': return gDot;
      case '/': return gSlash;
      case '-': return gDash;
      case '%': return gPercent;
      case ':': return gColon;
      case '0': return g0; case '1': return g1; case '2': return g2; case '3': return g3; case '4': return g4;
      case '5': return g5; case '6': return g6; case '7': return g7; case '8': return g8; case '9': return g9;
      case 'A': return gA; case 'B': return gB; case 'C': return gC; case 'D': return gD; case 'E': return gE;
      case 'F': return gF; case 'G': return gG; case 'H': return gH; case 'I': return gI; case 'K': return gK;
      case 'L': return gL; case 'M': return gM; case 'N': return gN; case 'O': return gO; case 'P': return gP; case 'Q': return gQ;
      case 'R': return gR; case 'S': return gS; case 'T': return gT; case 'U': return gU; case 'V': return gV;
      case 'W': return gW; case 'X': return gX; case 'Y': return gY; case 'Z': return gZ;
      default: return gSpace;
    }
  }

  char normalizeGlyphChar(char c) {
    if (c >= 'a' && c <= 'z') return static_cast<char>(c - 32);
    return c;
  }

  void drawBitmapText(int x, int y, const String& text, int scale, uint16_t fg, uint16_t bg) {
    int cx = x;
    for (size_t i = 0; i < text.length(); ++i) {
      char ch = static_cast<char>(text[i]);
      if (static_cast<unsigned char>(ch) >= 128) continue;
      ch = normalizeGlyphChar(ch);
      const uint8_t* glyph = glyph5x7(ch);
      for (int row = 0; row < 7; ++row) {
        for (int col = 0; col < 5; ++col) {
          const bool on = (glyph[row] & (1 << (4 - col))) != 0;
          tft.fillRect(cx + col * scale, y + row * scale, scale, scale, on ? fg : bg);
        }
      }
      cx += (6 * scale);
    }
  }


  String diagnosticValueForLabel(const char* label) {
    if (label == nullptr) return "123";
    String text(label);
    if (text == "Geschwindigkeit") return "123 km/h";
    if (text == "Drehzahl") return "2500 rpm";
    if (text == "Kühlmittel") return "92 C";
    if (text == "Bordspannung") return "13.8 V";
    if (text == "Öltemperatur") return "101 C";
    if (text == "Motorlast") return "37 %";
    if (text == "Ansaugluft") return "28 C";
    if (text == "Ø Verbrauch") return "7.2 L/100";
    if (text == "Kraftstoffrate") return "4.6 L/h";
    if (text == "Drosselklappe") return "18 %";
    if (text == "MAF") return "14.8 g/s";
    if (text == "Tankfüllstand") return "64 %";
    if (text == "Motorlaufzeit") return "845 s";
    if (text == "Außentemp.") return "24 C";
    if (text == "CAN Frames") return "128 frames";
    if (text == "CAN Status") return "AKTIV";
    if (text == "ESP-NOW") return "aktiv";
    if (text == "CAN/OBD") return "OK";
    if (text == "Datenqualität") return "99 %";
    if (text == "CRC/Drop") return "0/0";
    if (text == "Fehlercodes / DTC") return "Keine";
    return "123";
  }

  void drawSegH(int x, int y, int w, int t, uint16_t color) {
    tft.fillRoundRect(x, y, w, t, t / 2, color);
  }

  void drawSegV(int x, int y, int t, int h, uint16_t color) {
    tft.fillRoundRect(x, y, t, h, t / 2, color);
  }

  int segmentThickness(int scale) { return 2 * scale; }
  int segmentWidth(int scale) { return 8 * scale; }
  int segmentHeight(int scale) { return 8 * scale; }
  int segmentDigitWidth(int scale) { return segmentWidth(scale) + 2 * segmentThickness(scale); }
  int segmentTotalHeight(int scale) { return 2 * segmentHeight(scale) + 3 * segmentThickness(scale); }
  uint16_t segmentOnColor() { return DisplayConfig::Warn; }
  uint16_t segmentOffColor() { return DisplayConfig::Background; }

  void drawSevenSegmentDigit(int x, int y, int digit, int scale, uint16_t color, uint16_t offColor) {
    const int segT = segmentThickness(scale);
    const int segW = segmentWidth(scale);
    const int segH = segmentHeight(scale);

    const uint8_t maskTable[10] = {
      0b1111110, // 0
      0b0110000, // 1
      0b1101101, // 2
      0b1111001, // 3
      0b0110011, // 4
      0b1011011, // 5
      0b1011111, // 6
      0b1110000, // 7
      0b1111111, // 8
      0b1111011  // 9
    };

    uint8_t mask = 0;
    if (digit >= 0 && digit <= 9) mask = maskTable[digit];

    const uint16_t cA = (mask & 0b1000000) ? color : offColor;
    const uint16_t cB = (mask & 0b0100000) ? color : offColor;
    const uint16_t cC = (mask & 0b0010000) ? color : offColor;
    const uint16_t cD = (mask & 0b0001000) ? color : offColor;
    const uint16_t cE = (mask & 0b0000100) ? color : offColor;
    const uint16_t cF = (mask & 0b0000010) ? color : offColor;
    const uint16_t cG = (mask & 0b0000001) ? color : offColor;

    // a
    drawSegH(x + segT, y, segW, segT, cA);
    // b
    drawSegV(x + segT + segW, y + segT, segT, segH, cB);
    // c
    drawSegV(x + segT + segW, y + segT + segH + segT, segT, segH, cC);
    // d
    drawSegH(x + segT, y + 2 * segH + 2 * segT, segW, segT, cD);
    // e
    drawSegV(x, y + segT + segH + segT, segT, segH, cE);
    // f
    drawSegV(x, y + segT, segT, segH, cF);
    // g
    drawSegH(x + segT, y + segT + segH, segW, segT, cG);
  }

  void drawSevenSegmentNumber(int x, int y, int value, int digits, int scale, uint16_t color, uint16_t offColor) {
    int divisor = 1;
    for (int i = 1; i < digits; ++i) divisor *= 10;

    const int digitW = segmentDigitWidth(scale);
    const int spacing = 3 * scale;
    int remaining = value;

    for (int i = 0; i < digits; ++i) {
      const int d = remaining / divisor;
      remaining %= divisor;
      if (divisor > 1) divisor /= 10;
      drawSevenSegmentDigit(x + i * (digitW + spacing), y, d, scale, color, offColor);
    }
  }

  void drawDiagnosticSegmentsInBox(int x, int y, int w, int h, const char* label) {
    int number = 123;
    int digits = 3;

    if (label != nullptr) {
      String text(label);
      if (text == "Drehzahl") {
        number = 2500;
        digits = 4;
      } else if (text == "Bordspannung") {
        number = 138;
        digits = 3;
      } else if (text == "Kühlmittel") {
        number = 92;
        digits = 2;
      }
    }

    const int scale = 1;
    const int digitW = segmentDigitWidth(scale);
    const int spacing = 3 * scale;
    const int totalW = digits * digitW + (digits - 1) * spacing;
    const int totalH = segmentTotalHeight(scale);
    const int startX = x + (w - totalW) / 2;
    const int startY = y + (h - totalH) / 2 + 8;

    drawSevenSegmentNumber(startX, startY, number, digits, scale, segmentOnColor(), segmentOffColor());
  }

  void drawFooter() {
    tft.fillRect(0, tft.height() - 16, tft.width(), 16, DisplayConfig::Background);
    tft.setTextSize(1);
    tft.setTextDatum(ML_DATUM);
    tft.setTextColor(DisplayConfig::Muted, DisplayConfig::Background);
    String footer = "FW ";
    footer += DISPLAY_FIRMWARE_VERSION;
    footer += "  Proto ";
    footer += String(TELEMETRY_PROTOCOL_VERSION);
    tft.drawString(footer, 4, tft.height() - 8);
  }

  bool isMissingDisplayValue(const String& value) {
    String normalized = value;
    normalized.trim();
    normalized.toUpperCase();
    return normalized.length() == 0 ||
           normalized == "--" ||
           normalized == "N/A" ||
           normalized == "NA" ||
           normalized == "UNKNOWN" ||
           normalized == "UNBEKANNT";
  }

  String textOrFallback(const char* name, const char* fallback) {
    String value = DisplayData::displayText(name);
    return isMissingDisplayValue(value) ? String(fallback == nullptr ? "WARTET" : fallback) : value;
  }

  String valueOrFallback(const char* name, uint8_t decimals, const char* fallback) {
    String value = DisplayData::displayValue(name, decimals);
    return isMissingDisplayValue(value) ? String(fallback == nullptr ? "WARTET" : fallback) : value;
  }

  String countOrZero(const char* name) {
    String value = DisplayData::displayValue(name, 0);
    return isMissingDisplayValue(value) ? "0" : value;
  }

  String canIdFromRaw(const String& raw) {
    if (isMissingDisplayValue(raw)) return "--";
    const int spaceIndex = raw.indexOf(' ');
    return spaceIndex > 0 ? raw.substring(0, spaceIndex) : raw;
  }

  String compactCanHex(const String& raw) {
    if (isMissingDisplayValue(raw)) return "WARTET AUF CAN";
    String text = raw;
    text.replace("DLC", " D");
    return text;
  }

  void drawMetricBox(int x, int y, int w, int h, const char* label, const String& value, uint16_t color) {
    const bool overlay = startupOverlayActive();
    const String shownValue = overlay ? diagnosticValueForLabel(label) : value;
    const bool missing = !overlay && isMissingDisplayValue(shownValue);
    const uint16_t effectiveColor = missing ? DisplayConfig::Muted : color;

    if (fullPageRedraw) {
      tft.fillRoundRect(x, y, w, h, 6, DisplayConfig::Panel);

      String labelText = String(label == nullptr ? "" : label);
      labelText.toUpperCase();
      drawBitmapText(x + 8, y + 7, labelText, 1, DisplayConfig::Text, DisplayConfig::Panel);
    }

    tft.drawRoundRect(x, y, w, h, 6, effectiveColor);
    tft.fillRoundRect(x + 2, y + h - 5, w - 4, 3, 2, effectiveColor);

    // Subtle value background for readability without changing the overall card style.
    const int valueBgX = x + 6;
    const int valueBgY = y + 24;
    const int valueBgW = w - 12;
    const int valueBgH = h - 28;
    tft.fillRoundRect(valueBgX, valueBgY, valueBgW, valueBgH, 4, DisplayConfig::Background);

    if (overlay) {
      drawDiagnosticSegmentsInBox(x, y, w, h, label);
      return;
    }

    String textValue = missing ? "WARTET" : shownValue;
    const uint16_t valueTextColor = missing ? DisplayConfig::Muted : effectiveColor;
    int valueScale = 2;
    int maxChars = (valueBgW - 8) / (6 * valueScale);
    if (maxChars <= 3 || textValue.length() > static_cast<size_t>(maxChars)) {
      valueScale = 1;
      maxChars = (valueBgW - 8) / (6 * valueScale);
    }
    if (maxChars > 3 && textValue.length() > static_cast<size_t>(maxChars)) {
      textValue = textValue.substring(0, maxChars - 3) + "...";
    }

    const int textH = 7 * valueScale;
    const int valueTextX = valueBgX + 4;
    const int valueTextY = valueBgY + (valueBgH - textH) / 2;
    String normalized = textValue;
    normalized.toUpperCase();
    drawBitmapText(valueTextX, valueTextY, normalized, valueScale, valueTextColor, DisplayConfig::Background);
  }

  void drawSmallStatusCell(int x, int y, int w, const char* label, const String& value, uint16_t color) {
    constexpr int cellHeight = 16;
    const bool missing = isMissingDisplayValue(value);
    const uint16_t effectiveColor = missing ? DisplayConfig::Muted : color;
    tft.fillRoundRect(x, y, w, cellHeight, 5, DisplayConfig::Panel);
    tft.drawRoundRect(x, y, w, cellHeight, 5, effectiveColor);
    String text = String(label == nullptr ? "" : label);
    text += ":";
    text += missing ? "WARTET" : value;
    text.toUpperCase();
    const int maxChars = (w - 8) / 6;
    if (maxChars > 3 && text.length() > static_cast<size_t>(maxChars)) {
      text = text.substring(0, maxChars);
    }
    drawBitmapText(x + 4, y + 5, text, 1, effectiveColor, DisplayConfig::Panel);
  }

  void drawInfoLine(int y, const char* label, const String& value, uint16_t color = DisplayConfig::Text) {
    tft.setTextDatum(ML_DATUM);
    tft.setTextSize(1);
    tft.setTextColor(DisplayConfig::Muted, DisplayConfig::Panel);
    tft.drawString(label == nullptr ? "" : label, 14, y);
    tft.setTextColor(color, DisplayConfig::Panel);
    String text = value;
    if (text.length() > 31) text = text.substring(0, 31) + "...";
    tft.drawString(text, 112, y);
  }

  String ageText(uint32_t timestampMs) {
    if (timestampMs == 0) return "--";
    return String((millis() - timestampMs) / 1000) + "s";
  }

  uint16_t statusColorForValue(DisplayTelemetryValue* value) {
    if (value == nullptr) return DisplayConfig::Muted;
    if (!DisplayData::isFresh(value)) return DisplayConfig::Muted;
    String status = value->status;
    status.toUpperCase();
    if (status == "ERROR" || status == "TIMEOUT" || status == "SEND_FAIL") return DisplayConfig::Error;
    if (status == "WARN" || status == "UNSUPPORTED") return DisplayConfig::Warn;
    return DisplayConfig::Ok;
  }

  uint8_t visiblePageCount() {
    return DisplayData::runtime().diagnosticPagesEnabled ? DisplayConfig::PageCount : DisplayConfig::NormalPageCount;
  }

  uint8_t pageIdForCurrentPage() {
    const uint8_t count = visiblePageCount();
    if (count == 0) return DisplayConfig::MainPageIndex;
    if (DisplayData::currentPage >= count) DisplayData::currentPage = DisplayConfig::MainPageIndex;
    return DisplayData::currentPage;
  }

  void setDiagnosticPagesEnabled(bool enabled) {
    DisplayData::runtime().diagnosticPagesEnabled = enabled;
    if (!enabled && DisplayData::currentPage >= DisplayConfig::NormalPageCount) {
      DisplayData::currentPage = DisplayConfig::MainPageIndex;
    }
    DisplayData::lastRenderedPage = 255;
    DisplayData::markDirty();
    DiagnosticLog::appendf("[display-ui] diagnostic pages %s", enabled ? "enabled" : "disabled");
  }

  void drawMainPageV2() {
    using namespace DisplayData;
    drawMetricBox(6, 24, 150, 52, "Geschwindigkeit", displayValue("Speed", 0), valueColor("Speed"));
    drawMetricBox(164, 24, 150, 52, "Drehzahl", displayValue("RPM", 0), valueColor("RPM"));
    drawMetricBox(6, 82, 150, 52, "Kuehlmittel", displayValue("CoolantTemp", 0), valueColor("CoolantTemp"));
    drawMetricBox(164, 82, 150, 52, "Bordspannung", displayValue("BatteryVoltage", 1), valueColor("BatteryVoltage"));

    DisplayTelemetryValue* canStatus = findValue("CAN");
    DisplayTelemetryValue* obdStatus = findValue("OBD");
    DisplayTelemetryValue* dtc = findValue("DTC");
    const bool hasFault = dtc != nullptr && isFresh(dtc) && dtc->status == "WARN" && dtc->value != "Keine";
    drawSmallStatusCell(6, 136, 74, "ESP", isEspNowConnected() ? "OK" : "ERR",
                        isEspNowConnected() ? DisplayConfig::Ok : DisplayConfig::Error);
    drawSmallStatusCell(84, 136, 74, "CAN", isCanStatusRecent() ? textOrFallback("CAN", "WARTET") : "WARTET",
                        statusColorForValue(canStatus));
    drawSmallStatusCell(162, 136, 74, "OBD", isObdStatusRecent() ? textOrFallback("OBD", "WARTET") : "WARTET",
                        statusColorForValue(obdStatus));
    drawSmallStatusCell(240, 136, 74, "DTC", hasFault ? "AKT" : (isFresh(dtc) ? "OK" : "KEINE"),
                        hasFault ? DisplayConfig::Warn : (isFresh(dtc) ? DisplayConfig::Ok : DisplayConfig::Muted));
  }

  void drawEnginePageV2() {
    using namespace DisplayData;
    drawMetricBox(6, 24, 150, 48, "Oeltemp.", displayValue("OilTemp", 0), valueColor("OilTemp"));
    drawMetricBox(164, 24, 150, 48, "Kuehlmittel", displayValue("CoolantTemp", 0), valueColor("CoolantTemp"));
    drawMetricBox(6, 78, 150, 48, "Motorlast", displayValue("EngineLoad", 0), valueColor("EngineLoad"));
    drawMetricBox(164, 78, 150, 48, "Ansaugluft", displayValue("IntakeTemp", 0), valueColor("IntakeTemp"));
    drawSmallStatusCell(6, 134, 100, "MAP", valueOrFallback("ManifoldAbsolutePressure", 0, "WARTET"), valueColor("ManifoldAbsolutePressure"));
    drawSmallStatusCell(110, 134, 100, "MAF", valueOrFallback("MAF", 1, "WARTET"), valueColor("MAF"));
    drawSmallStatusCell(214, 134, 100, "DK", valueOrFallback("Throttle", 0, "WARTET"), valueColor("Throttle"));
  }

  void drawConsumptionPageV2() {
    using namespace DisplayData;
    drawMetricBox(6, 24, 150, 48, "Momentan", displayValue("InstantConsumption", 1), valueColor("InstantConsumption"));
    drawMetricBox(164, 24, 150, 48, "Durchschnitt", displayValue("AverageConsumption", 1), valueColor("AverageConsumption"));
    drawMetricBox(6, 78, 150, 48, "Kraftstoff", displayValue("FuelRate", 1), valueColor("FuelRate"));
    drawMetricBox(164, 78, 150, 48, "Tempo", displayValue("Speed", 0), valueColor("Speed"));
    drawSmallStatusCell(6, 134, 150, "Laufzeit", valueOrFallback("RunTime", 0, "WARTET"), valueColor("RunTime"));
    drawSmallStatusCell(164, 134, 150, "Tank", valueOrFallback("FuelLevel", 0, "WARTET"), valueColor("FuelLevel"));
  }

  void drawDiagnosticsPageV2() {
    using namespace DisplayData;
    const uint32_t totalPackets = runtime().receivedPackets + runtime().droppedPackets;
    const uint8_t quality = totalPackets == 0 ? 0 : (runtime().receivedPackets * 100UL) / totalPackets;
    DisplayTelemetryValue* canStatus = findValue("CAN");
    DisplayTelemetryValue* obdStatus = findValue("OBD");
    DisplayTelemetryValue* heartbeat = findValue("HEARTBEAT");

    drawMetricBox(6, 24, 100, 38, "ESP-NOW", isEspNowConnected() ? "OK" : "ERR",
                  isEspNowConnected() ? DisplayConfig::Ok : DisplayConfig::Error);
    drawMetricBox(110, 24, 100, 38, "CAN", isCanStatusRecent() ? textOrFallback("CAN", "WARTET") : "WARTET",
                  statusColorForValue(canStatus));
    drawMetricBox(214, 24, 100, 38, "OBD", isObdStatusRecent() ? textOrFallback("OBD", "WARTET") : "WARTET",
                  statusColorForValue(obdStatus));
    drawMetricBox(6, 68, 150, 38, "Update", ageText(runtime().lastReceivedAt),
                  isEspNowConnected() ? DisplayConfig::Ok : DisplayConfig::Error);
    drawMetricBox(164, 68, 150, 38, "Heartbeat", heartbeat != nullptr ? heartbeat->value : "0",
                  isFresh(heartbeat) ? DisplayConfig::Ok : DisplayConfig::Warn);

    tft.fillRoundRect(6, 112, 308, 38, 6, DisplayConfig::Panel);
    drawInfoLine(121, "Pakete", String(runtime().receivedPackets) + "/" + String(runtime().droppedPackets) +
                           " Q " + String(quality) + "%");
    drawInfoLine(137, "Firmware", String(DISPLAY_FIRMWARE_VERSION) + " Seq " + String(runtime().lastSequence));
  }

  void drawUdsDtcPageV2() {
    using namespace DisplayData;
    DisplayTelemetryValue* dtc = findValue("DTC");
    DisplayTelemetryValue* udsStatus = findValue("UDS");
    DisplayTelemetryValue* udsDtc = findValue("UDS_DTC");
    const bool dtcFresh = isFresh(dtc);
    const bool hasFault = dtcFresh && dtc->status == "WARN" && dtc->value != "Keine";

    drawMetricBox(6, 24, 100, 38, "UDS", textOrFallback("UDS", "WARTET"), statusColorForValue(udsStatus));
    drawMetricBox(110, 24, 100, 38, "ECUs", countOrZero("ReachableEcus"), valueColor("ReachableEcus"));
    drawMetricBox(214, 24, 100, 38, "Pending", countOrZero("UdsPending"), valueColor("UdsPending"));

    tft.fillRoundRect(6, 68, 308, 80, 6, DisplayConfig::Panel);
    String vin = displayText("VIN");
    if (vin == "--") vin = textOrFallback("UDS_VIN", "WARTET");
    String dtcText = displayText("DTC");
    String udsDtcText = displayText("UDS_DTC");
    String nrc = displayText("UDS_NRC");
    if (dtcText == "--") dtcText = "KEINE DATEN";
    if (udsDtcText == "--") udsDtcText = "KEINE DATEN";
    if (nrc == "--") nrc = udsStatus != nullptr ? udsStatus->value : "KEIN NRC";
    drawInfoLine(78, "VIN", vin);
    drawInfoLine(96, "OBD-DTC", dtcText, hasFault ? DisplayConfig::Warn : (dtcFresh ? DisplayConfig::Ok : DisplayConfig::Muted));
    drawInfoLine(114, "UDS-DTC", udsDtcText, statusColorForValue(udsDtc));
    drawInfoLine(130, "NRC/Backoff", nrc + " / " + textOrFallback("UdsBackoff", "0s"),
                 nrc.indexOf("0x78") >= 0 ? DisplayConfig::Warn : DisplayConfig::Text);
  }

  void drawCanSnifferPageV2() {
    using namespace DisplayData;
    const String raw = displayText("LastCAN");
    const String hint = textOrFallback("CANHint", "NOCH KEIN HINWEIS");
    drawMetricBox(6, 24, 100, 38, "Sniffer", textOrFallback("CanSniffer", isMissingDisplayValue(raw) ? "WARTET" : "RAW"),
                  isMissingDisplayValue(raw) ? DisplayConfig::Muted : DisplayConfig::Ok);
    drawMetricBox(110, 24, 100, 38, "Baseline", textOrFallback("CanBaseline", "LIVE"),
                  isMissingDisplayValue(raw) ? DisplayConfig::Muted : DisplayConfig::Ok);
    drawMetricBox(214, 24, 100, 38, "Frames", valueOrFallback("CANCount", 0, String(runtime().receivedPackets).c_str()), valueColor("CANCount"));

    tft.fillRoundRect(6, 68, 308, 80, 6, DisplayConfig::Panel);
    drawInfoLine(78, "Kandidaten", textOrFallback("CanCandidates", "RAW-HEX AKTIV"));
    drawInfoLine(96, "CAN-ID", textOrFallback("CanCandidateId", canIdFromRaw(raw).c_str()));
    drawInfoLine(114, "Letzter", compactCanHex(raw));
    drawInfoLine(130, "Hinweis", hint, DisplayConfig::Accent);
  }

  void drawAdditionalPageV2() {
    using namespace DisplayData;
    drawMetricBox(6, 24, 150, 48, "MAF", displayValue("MAF", 1), valueColor("MAF"));
    drawMetricBox(164, 24, 150, 48, "MAP", displayValue("ManifoldAbsolutePressure", 0), valueColor("ManifoldAbsolutePressure"));
    drawMetricBox(6, 78, 150, 48, "BARO", displayValue("BarometricPressure", 0), valueColor("BarometricPressure"));
    drawMetricBox(164, 78, 150, 48, "Aussentemp.", displayValue("AmbientTemp", 0), valueColor("AmbientTemp"));
    drawSmallStatusCell(6, 134, 150, "ECU V", valueOrFallback("EcuVoltage", 1, "WARTET"), valueColor("EcuVoltage"));
    drawSmallStatusCell(164, 134, 150, "Runtime", valueOrFallback("RunTime", 0, "WARTET"), valueColor("RunTime"));
  }

  void drawPowerPageV2() {
    using namespace DisplayData;
    const Runtime::DisplayRuntimeState& state = runtime();
    drawMetricBox(6, 24, 150, 48, "Vehicle", state.vehicleState, state.displaySleepRequested ? DisplayConfig::Warn : DisplayConfig::Ok);
    drawMetricBox(164, 24, 150, 48, "Score", String(state.activityScore), state.activityScore >= 10 ? DisplayConfig::Ok : (state.activityScore > 0 ? DisplayConfig::Warn : DisplayConfig::Muted));
    drawMetricBox(6, 78, 150, 48, "Command", state.powerCommand, state.powerCommand == "Sleep" ? DisplayConfig::Warn : DisplayConfig::Ok);
    drawMetricBox(164, 78, 150, 48, "Display", displaySleeping ? "Sleep" : "Running", displaySleeping ? DisplayConfig::Muted : DisplayConfig::Ok);
    drawSmallStatusCell(6, 134, 150, "Last CAN", ageText(state.lastCanStatusAt), isCanStatusRecent() ? DisplayConfig::Ok : DisplayConfig::Warn);
    drawSmallStatusCell(164, 134, 150, "Last OBD", ageText(state.lastObdStatusAt), isObdStatusRecent() ? DisplayConfig::Ok : DisplayConfig::Warn);
  }

  void renderCurrentPage() {
    using namespace DisplayData;
    const bool pageChanged = (lastRenderedPage != currentPage);
    fullPageRedraw = pageChanged;
    if (pageChanged) {
      // Full clear only on page transitions to avoid visible flicker on each telemetry update.
      tft.fillScreen(DisplayConfig::Background);
    }
    drawStatusBar();

    switch (pageIdForCurrentPage()) {
      case 0: drawMainPageV2(); break;
      case 1: drawEnginePageV2(); break;
      case 2: drawConsumptionPageV2(); break;
      case 3: drawDiagnosticsPageV2(); break;
      case 4: drawUdsDtcPageV2(); break;
      case 5: drawCanSnifferPageV2(); break;
      case 6: drawAdditionalPageV2(); break;
      case 7: drawPowerPageV2(); break;
    }

    drawFooter();
  }

  void showBootScreen() {
    logUi("drawing boot screen");
    tft.fillScreen(DisplayConfig::Background);
    tft.setTextDatum(MC_DATUM);
    tft.setTextSize(3);
    tft.setTextColor(DisplayConfig::Accent, DisplayConfig::Background);
    tft.drawString("CAN OBD2", tft.width() / 2, tft.height() / 2 - 12);
    tft.setTextSize(1);
    tft.setTextColor(DisplayConfig::Muted, DisplayConfig::Background);
    tft.drawString("Display " DISPLAY_FIRMWARE_VERSION, tft.width() / 2, tft.height() / 2 + 22);
    delay(900);
  }
}

namespace DisplayUi {
  void begin() {
    logUi("begin");
    pinMode(DisplayConfig::NextPageButtonPin, INPUT_PULLUP);
    logPinState("button", DisplayConfig::NextPageButtonPin);
    setupDisplayPower();
    logUi("calling tft.init");
    tft.init();
    logUi("tft.init done");
    applyPanelRevisionInit();
    tft.setRotation(DisplayConfig::Rotation);
    logUi("rotation set");
    DiagnosticLog::appendf("[display-ui] size %dx%d", tft.width(), tft.height());
    setupBacklight();
    showBootScreen();
    logUi("begin complete");
  }

  void handleButton() {
    using namespace DisplayData;
    const uint32_t now = millis();
    const bool pressed = digitalRead(DisplayConfig::NextPageButtonPin) == LOW;
    if (pressed != lastButtonPressed) {
      DiagnosticLog::appendf("[display-ui] button %s", pressed ? "pressed" : "released");
      lastButtonPressed = pressed;

      if (pressed) {
        runtime().displaySleepRequested = false;
        buttonPressedAt = now;
        longPressHandled = false;
      } else if (!longPressHandled && now - buttonPressedAt >= DisplayConfig::ButtonDebounceMs) {
        const uint32_t pressDuration = now - buttonPressedAt;
        if (pressDuration >= DisplayConfig::VeryLongPressMs) {
          setDiagnosticPagesEnabled(!runtime().diagnosticPagesEnabled);
          currentPage = DisplayConfig::MainPageIndex;
        } else if (pressDuration >= DisplayConfig::LongPressMs) {
          currentPage = DisplayConfig::MainPageIndex;
          lastRenderedPage = 255;
          markDirty();
          DiagnosticLog::appendf("[display-ui] long press page -> %u", static_cast<unsigned int>(currentPage));
        } else {
          currentPage = (currentPage + 1) % visiblePageCount();
          lastRenderedPage = 255;
          markDirty();
          DiagnosticLog::appendf("[display-ui] page -> %u", static_cast<unsigned int>(currentPage));
        }
        lastRenderedPage = 255;
        lastButtonAt = now;
        markDirty();
      }
    }
  }

  void renderIfDue() {
    using namespace DisplayData;
    const uint32_t now = millis();

    if (runtime().displaySleepRequested) {
      if (!displaySleeping) {
        setBacklight(DisplayConfig::BacklightSleep);
        displaySleeping = true;
        DiagnosticLog::appendf("[display-ui] power command sleep");
      }
      return;
    }

    if (displaySleeping) {
      setBacklight(DisplayConfig::BacklightOn);
      displaySleeping = false;
      lastRenderedPage = 255;
      markDirty();
      DiagnosticLog::appendf("[display-ui] power command wakeup");
    }

    const bool periodicRender = (now - lastScreenRefresh) >= DisplayConfig::ForceFullRenderMs;
    if ((renderDirty && now - lastScreenRefresh >= DisplayConfig::ScreenRefreshMs) ||
        lastRenderedPage != currentPage ||
        periodicRender) {
      renderCurrentPage();
      lastRenderedPage = currentPage;
      lastScreenRefresh = now;
      renderDirty = false;

      if (LoggingConfig::SerialEnabled && now - lastUiStatsLogMs >= 5000) {
        DiagnosticLog::appendf("[display-ui] page=%u values=%u connected=%s",
                               static_cast<unsigned int>(currentPage),
                               static_cast<unsigned int>(valueCount),
                               isConnected() ? "yes" : "no");
        lastUiStatsLogMs = now;
      }
    }
  }
}
