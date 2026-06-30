#include "DisplayUi.h"
#include "DisplayData.h"
#include "driver/ledc.h"

namespace {
  TFT_eSPI tft = TFT_eSPI();
  bool lastButtonPressed = false;
  uint32_t buttonPressedAt = 0;
  bool longPressHandled = false;
  uint32_t lastUiStatsLogMs = 0;
  bool fullPageRedraw = true;

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
    Serial.print("[display-ui] ");
    Serial.println(message);
  }

  void logPinState(const char* label, int pin) {
    Serial.print("[display-ui] ");
    Serial.print(label);
    Serial.print(" pin ");
    Serial.print(pin);
    Serial.print(" state=");
    Serial.println(digitalRead(pin));
  }

  void applyPanelRevisionInit() {
    logUi("applying panel revision init");
    for (const DisplayInitCommand& entry : kDisplayInitSequence) {
      tft.writecommand(entry.command);
      for (uint8_t index = 0; index < (entry.length & 0x7F); ++index) {
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
    const uint16_t barColor = isConnected() ? DisplayConfig::Panel : DisplayConfig::Error;
    tft.fillRect(0, 0, tft.width(), 20, barColor);
    tft.setTextSize(1);
    tft.setTextDatum(ML_DATUM);
    tft.setTextColor(DisplayConfig::Text, barColor);
    tft.drawString(isConnected() ? "ESP-NOW OK" : "VERBINDUNG VERLOREN", 4, 10);

    tft.setTextDatum(MR_DATUM);
    String right = "S" + String(currentPage + 1) + "/" + String(DisplayConfig::PageCount);
    if (DisplayConfig::EnableStartupValueOverlay && millis() < DisplayConfig::StartupValueOverlayMs) {
      right += " DIAG";
    }
    if (lastReceivedAt > 0) right += "  " + String((millis() - lastReceivedAt) / 1000) + "s";
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
    static const uint8_t gR[7] = {0b11110,0b10001,0b10001,0b11110,0b10100,0b10010,0b10001};
    static const uint8_t gS[7] = {0b01111,0b10000,0b10000,0b01110,0b00001,0b00001,0b11110};
    static const uint8_t gT[7] = {0b11111,0b00100,0b00100,0b00100,0b00100,0b00100,0b00100};
    static const uint8_t gU[7] = {0b10001,0b10001,0b10001,0b10001,0b10001,0b10001,0b01110};
    static const uint8_t gV[7] = {0b10001,0b10001,0b10001,0b10001,0b10001,0b01010,0b00100};
    static const uint8_t gW[7] = {0b10001,0b10001,0b10001,0b10101,0b10101,0b10101,0b01010};
    static const uint8_t gY[7] = {0b10001,0b10001,0b01010,0b00100,0b00100,0b00100,0b00100};

    switch (c) {
      case ' ': return gSpace;
      case '.': return gDot;
      case '/': return gSlash;
      case '-': return gDash;
      case '%': return gPercent;
      case '0': return g0; case '1': return g1; case '2': return g2; case '3': return g3; case '4': return g4;
      case '5': return g5; case '6': return g6; case '7': return g7; case '8': return g8; case '9': return g9;
      case 'A': return gA; case 'B': return gB; case 'C': return gC; case 'D': return gD; case 'E': return gE;
      case 'F': return gF; case 'G': return gG; case 'H': return gH; case 'I': return gI; case 'K': return gK;
      case 'L': return gL; case 'M': return gM; case 'N': return gN; case 'O': return gO; case 'P': return gP;
      case 'R': return gR; case 'S': return gS; case 'T': return gT; case 'U': return gU; case 'V': return gV;
      case 'W': return gW; case 'Y': return gY;
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

  void drawMetricBox(int x, int y, int w, int h, const char* label, const String& value, uint16_t color) {
    if (fullPageRedraw) {
      tft.fillRoundRect(x, y, w, h, 6, DisplayConfig::Panel);

      String labelText = String(label == nullptr ? "" : label);
      labelText.toUpperCase();
      drawBitmapText(x + 8, y + 7, labelText, 1, DisplayConfig::Text, DisplayConfig::Panel);
    }

    tft.drawRoundRect(x, y, w, h, 6, color);
    tft.fillRoundRect(x + 2, y + h - 5, w - 4, 3, 2, color);

    // Subtle value background for readability without changing the overall card style.
    const int valueBgX = x + 6;
    const int valueBgY = y + 24;
    const int valueBgW = w - 12;
    const int valueBgH = h - 28;
    tft.fillRoundRect(valueBgX, valueBgY, valueBgW, valueBgH, 4, DisplayConfig::Background);

    const bool overlay = startupOverlayActive();
    const String shownValue = overlay ? diagnosticValueForLabel(label) : value;

    if (overlay) {
      drawDiagnosticSegmentsInBox(x, y, w, h, label);
      return;
    }

    String textValue = (shownValue == "--") ? "N/A" : shownValue;
    const uint16_t valueTextColor = overlay ? DisplayConfig::Accent : color;
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

  void drawMainPage() {
    using namespace DisplayData;
    drawMetricBox(6, 28, 150, 58, "Geschwindigkeit", displayValue("Speed", 0), valueColor("Speed"));
    drawMetricBox(164, 28, 150, 58, "Drehzahl", displayValue("RPM", 0), valueColor("RPM"));
    drawMetricBox(6, 94, 150, 58, "Kühlmittel", displayValue("CoolantTemp", 0), valueColor("CoolantTemp"));
    drawMetricBox(164, 94, 150, 58, "Bordspannung", displayValue("BatteryVoltage", 1), valueColor("BatteryVoltage"));
  }

  void drawEnginePage() {
    using namespace DisplayData;
    drawMetricBox(6, 28, 150, 58, "Öltemperatur", displayValue("OilTemp", 0), valueColor("OilTemp"));
    drawMetricBox(164, 28, 150, 58, "Kühlmittel", displayValue("CoolantTemp", 0), valueColor("CoolantTemp"));
    drawMetricBox(6, 94, 150, 58, "Motorlast", displayValue("EngineLoad", 0), valueColor("EngineLoad"));
    drawMetricBox(164, 94, 150, 58, "Ansaugluft", displayValue("IntakeTemp", 0), valueColor("IntakeTemp"));
  }

  void drawConsumptionPage() {
    using namespace DisplayData;
    drawMetricBox(6, 28, 150, 58, "Ø Verbrauch", displayValue("AverageConsumption", 1), valueColor("AverageConsumption"));
    drawMetricBox(164, 28, 150, 58, "Kraftstoffrate", displayValue("FuelRate", 1), valueColor("FuelRate"));
    drawMetricBox(6, 94, 150, 58, "Geschwindigkeit", displayValue("Speed", 0), valueColor("Speed"));
    drawMetricBox(164, 94, 150, 58, "Drosselklappe", displayValue("Throttle", 0), valueColor("Throttle"));
  }

  void drawAdditionalPage() {
    using namespace DisplayData;
    drawMetricBox(6, 28, 150, 58, "MAF", displayValue("MAF", 1), valueColor("MAF"));
    drawMetricBox(164, 28, 150, 58, "Tankfüllstand", displayValue("FuelLevel", 0), valueColor("FuelLevel"));
    drawMetricBox(6, 94, 150, 58, "Motorlaufzeit", displayValue("RunTime", 0), valueColor("RunTime"));
    drawMetricBox(164, 94, 150, 58, "Außentemp.", displayValue("AmbientTemp", 0), valueColor("AmbientTemp"));
  }

  void drawDiagnosticsPage() {
    using namespace DisplayData;
    uint32_t totalPackets = receivedPackets + droppedPackets;
    uint8_t quality = totalPackets == 0 ? 0 : (receivedPackets * 100UL) / totalPackets;
    DisplayTelemetryValue* canStatus = findValue("CAN");

    drawMetricBox(6, 28, 150, 42, "ESP-NOW", isConnected() ? "aktiv" : "verloren", isConnected() ? DisplayConfig::Ok : DisplayConfig::Error);
    bool canStatusRecent = canStatus != nullptr && millis() - canStatus->updatedAt <= DisplayConfig::ValueTimeoutMs;
    drawMetricBox(164, 28, 150, 42, "CAN/OBD", canStatusRecent ? canStatus->value : "--", isFresh(canStatus) ? DisplayConfig::Ok : DisplayConfig::Warn);
    drawMetricBox(6, 76, 150, 42, "Datenqualität", String(quality) + " %", quality > 90 ? DisplayConfig::Ok : DisplayConfig::Warn);
    drawMetricBox(164, 76, 150, 42, "CRC/Drop", String(crcErrors) + "/" + String(droppedPackets), (crcErrors == 0 && droppedPackets == 0) ? DisplayConfig::Ok : DisplayConfig::Warn);

    tft.fillRoundRect(6, 124, 308, 28, 6, DisplayConfig::Panel);
    tft.setTextDatum(ML_DATUM);
    tft.setTextSize(1);
    tft.setTextColor(DisplayConfig::Muted, DisplayConfig::Panel);
    String firmwareLine = String("FW ") + DISPLAY_FIRMWARE_VERSION + "  Seq " + String(lastSequence);
    tft.drawString(firmwareLine, 14, 132);
    tft.setTextColor(DisplayConfig::Text, DisplayConfig::Panel);
    String sim = displayText("Simulation");
    String scenario = displayText("SimScenario");
    if (sim == "--") sim = "inaktiv";
    if (scenario == "--") scenario = "NormalSingleFrame";
    String line = "Sim " + sim + "  " + scenario;
    if (line.length() > 42) line = line.substring(0, 42) + "...";
    tft.drawString(line, 14, 144);
  }

  void drawCANPage() {
    using namespace DisplayData;
    drawMetricBox(6, 28, 150, 42, "CAN Frames", displayValue("CANCount", 0), valueColor("CANCount"));
    drawMetricBox(164, 28, 150, 42, "CAN Status", displayText("CAN"), isConnected() ? DisplayConfig::Ok : DisplayConfig::Warn);

    tft.fillRoundRect(6, 78, 308, 74, 6, DisplayConfig::Panel);
    tft.setTextDatum(TL_DATUM);
    tft.setTextSize(1);
    tft.setTextColor(DisplayConfig::Muted, DisplayConfig::Panel);
    tft.drawString("Letzter CAN-Frame:", 14, 86);
    tft.setTextColor(DisplayConfig::Text, DisplayConfig::Panel);
    String raw = displayText("LastCAN");
    if (raw.length() > 42) raw = raw.substring(0, 42) + "...";
    tft.drawString(raw, 14, 102);

    tft.setTextColor(DisplayConfig::Muted, DisplayConfig::Panel);
    tft.drawString("Auswertung:", 14, 126);
    tft.setTextColor(DisplayConfig::Accent, DisplayConfig::Panel);
    String hint = displayText("CANHint");
    if (hint.length() > 36) hint = hint.substring(0, 36) + "...";
    tft.drawString(hint, 14, 140);
  }

  void drawDTCPage() {
    using namespace DisplayData;
    DisplayTelemetryValue* dtc = findValue("DTC");
    const bool dtcFresh = dtc != nullptr && millis() - dtc->updatedAt <= DisplayConfig::ValueTimeoutMs;
    const bool hasFault = dtcFresh && dtc->status == "WARN" && dtc->value != "Keine";

    drawMetricBox(6, 28, 308, 48, "Fehlercodes / DTC", dtcFresh ? (hasFault ? "AKTIV" : "Keine") : "--",
                  hasFault ? DisplayConfig::Warn : (dtcFresh ? DisplayConfig::Ok : DisplayConfig::Muted));

    tft.fillRoundRect(6, 88, 308, 64, 6, DisplayConfig::Panel);
    tft.setTextDatum(TL_DATUM);
    tft.setTextSize(1);
    tft.setTextColor(DisplayConfig::Muted, DisplayConfig::Panel);
    tft.drawString("Codes:", 14, 96);

    tft.setTextSize(2);
    tft.setTextColor(hasFault ? DisplayConfig::Warn : DisplayConfig::Text, DisplayConfig::Panel);
    String codes = displayText("DTC");
    if (codes.length() > 24) codes = codes.substring(0, 24);
    tft.drawString(codes, 14, 116);

    tft.setTextSize(1);
    tft.setTextColor(DisplayConfig::Muted, DisplayConfig::Panel);
    String age = dtcFresh ? "Letzte DTC-Abfrage: " + String((millis() - dtc->updatedAt) / 1000) + "s" : "Keine aktuelle DTC-Abfrage";
    tft.drawString(age, 14, 142);
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

    switch (currentPage) {
      case 0: drawMainPage(); break;
      case 1: drawEnginePage(); break;
      case 2: drawConsumptionPage(); break;
      case 3: drawAdditionalPage(); break;
      case 4: drawCANPage(); break;
      case 5: drawDiagnosticsPage(); break;
      case 6: drawDTCPage(); break;
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
    Serial.printf("[display-ui] size %dx%d\n", tft.width(), tft.height());
    setupBacklight();
    showBootScreen();
    logUi("begin complete");
  }

  void handleButton() {
    using namespace DisplayData;
    const uint32_t now = millis();
    const bool pressed = digitalRead(DisplayConfig::NextPageButtonPin) == LOW;
    if (pressed != lastButtonPressed) {
      Serial.print("[display-ui] button ");
      Serial.println(pressed ? "pressed" : "released");
      lastButtonPressed = pressed;

      if (pressed) {
        buttonPressedAt = now;
        longPressHandled = false;
      } else if (!longPressHandled && now - buttonPressedAt >= DisplayConfig::ButtonDebounceMs) {
        currentPage = (currentPage + 1) % DisplayConfig::PageCount;
        lastRenderedPage = 255;
        lastButtonAt = now;
        markDirty();
        Serial.print("[display-ui] page -> ");
        Serial.println(currentPage);
      }
    }

    if (pressed &&
        !longPressHandled &&
        now - buttonPressedAt >= DisplayConfig::LongPressMs &&
        now - lastButtonAt > DisplayConfig::ButtonDebounceMs) {
      currentPage = DisplayConfig::MainPageIndex;
      lastRenderedPage = 255;
      lastButtonAt = now;
      longPressHandled = true;
      markDirty();
      Serial.print("[display-ui] long press page -> ");
      Serial.println(currentPage);
    }
  }

  void renderIfDue() {
    using namespace DisplayData;
    const uint32_t now = millis();
    const bool periodicRender = (now - lastScreenRefresh) >= DisplayConfig::ForceFullRenderMs;
    if ((renderDirty && now - lastScreenRefresh >= DisplayConfig::ScreenRefreshMs) ||
        lastRenderedPage != currentPage ||
        periodicRender) {
      renderCurrentPage();
      lastRenderedPage = currentPage;
      lastScreenRefresh = now;
      renderDirty = false;

      if (Config::Debug::Serial && now - lastUiStatsLogMs >= 5000) {
        Serial.printf("[display-ui] page=%u values=%u connected=%s\n",
                      static_cast<unsigned int>(currentPage),
                      static_cast<unsigned int>(valueCount),
                      isConnected() ? "yes" : "no");
        lastUiStatsLogMs = now;
      }
    }
  }
}
