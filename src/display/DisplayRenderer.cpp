#include "DisplayRenderer.h"
#include "DisplayData.h"
#include "DisplaySeverity.h"
#include "LiveValueView.h"

namespace {
using namespace DisplayCanvas;
using DisplayLogic::ValueState;
using DisplayLogic::ValueView;
using LiveData::Metric;

DisplayLogic::Smoother speedSmoother(DisplayConfig::FastValueSmoothingMs);
DisplayLogic::Smoother rpmSmoother(DisplayConfig::FastValueSmoothingMs);

// ---------------------------------------------------------------------------
// Value formatting
// ---------------------------------------------------------------------------

ValueView valueOf(Metric metric) {
  return DisplayData::live().get(metric, millis());
}

String formatNumber(float value, uint8_t decimals) {
  return String(static_cast<double>(value), static_cast<unsigned int>(decimals));
}

String formatValue(Metric metric, const ValueView& view) {
  if (view.state == ValueState::Unsupported) return "n.v.";
  if (!view.hasValue()) return "--";
  return formatNumber(view.value, LiveData::info(metric).decimals);
}

uint16_t colorFor(Metric metric, const ValueView& view) {
  if (view.state != ValueState::Fresh) return DisplayConfig::Muted;
  switch (DisplayLogic::severityForMetric(LiveData::info(metric).name, view.value, true)) {
    case DisplayLogic::DisplaySeverity::Critical: return DisplayConfig::Error;
    case DisplayLogic::DisplaySeverity::Warning: return DisplayConfig::Warn;
    default: return DisplayConfig::Text;
  }
}

// Shortens text with "..." until it fits into maxWidth pixels.
String fitText(const String& text, int maxWidth, uint8_t font) {
  if (frame.textWidth(text, font) <= maxWidth) return text;
  String shortened = text;
  while (shortened.length() > 1 && frame.textWidth(shortened + "...", font) > maxWidth) {
    shortened.remove(shortened.length() - 1);
  }
  return shortened + "...";
}

// ---------------------------------------------------------------------------
// Widgets (all draw into `frame`)
// ---------------------------------------------------------------------------

void panel(int x, int y, int w, int h, uint16_t accent) {
  frame.fillRoundRect(x, y, w, h, 6, DisplayConfig::Panel);
  frame.fillRect(x + 6, y + h - 3, w - 12, 3, accent);
}

void label(int x, int y, const char* text, uint8_t font = 2, uint16_t color = DisplayConfig::Muted) {
  frame.setTextDatum(TL_DATUM);
  frame.setTextColor(color);
  frame.drawString(text, x, y, font);
}

// Large value with label and unit; `display` overrides the shown number
// (used for the smoothed RPM/speed values).
void metricCard(int x, int y, int w, int h, Metric metric, bool big, const ValueView* display = nullptr) {
  const ValueView view = display != nullptr ? *display : valueOf(metric);
  const uint16_t color = colorFor(metric, view);
  const LiveData::MetricInfo& info = LiveData::info(metric);
  panel(x, y, w, h, view.state == ValueState::Fresh ? color : DisplayConfig::Panel);

  const String text = formatValue(metric, view);
  // Font 7 (7-segment, 48 px) only has digits, '.', '-' and ':'.
  const bool segmentFont = big && view.state != ValueState::Unsupported;
  const uint8_t font = segmentFont ? 7 : 4;

  if (h < 46) {
    // Low card (bottom rows): one small header line "Label unit" in font 1
    // (8 px) and the value below - a 16 px label would overlap the value.
    String head = info.label;
    if (info.unit[0] != '\0') head += String(" ") + info.unit;
    label(x + 6, y + 4, fitText(head, w - 12, 1).c_str(), 1);
    frame.setTextDatum(BR_DATUM);
    frame.setTextColor(color);
    frame.drawString(fitText(text, w - 12, font), x + w - 6, y + h - 5, font);
    return;
  }

  const bool narrow = w < 120;
  label(x + 8, y + 4, fitText(info.label, w - 16, 2).c_str());
  frame.setTextColor(DisplayConfig::Muted);
  if (narrow) {
    // Not enough room beside the label: unit goes bottom left, value bottom right.
    frame.setTextDatum(BL_DATUM);
    frame.drawString(info.unit, x + 8, y + h - 8, 2);
  } else {
    frame.setTextDatum(TR_DATUM);
    frame.drawString(info.unit, x + w - 8, y + 4, 2);
  }

  frame.setTextDatum(BR_DATUM);
  frame.setTextColor(color);
  const int unitSpace = narrow ? frame.textWidth(info.unit, 2) + 4 : 0;
  frame.drawString(fitText(text, w - 16 - unitSpace, font), x + w - 8, y + h - 7, font);
}

void textCell(int x, int y, int w, int h, const char* title, const String& value, uint16_t color) {
  panel(x, y, w, h, color == DisplayConfig::Muted ? DisplayConfig::Panel : color);
  label(x + 6, y + 4, title, 2);
  frame.setTextDatum(BR_DATUM);
  frame.setTextColor(color);
  frame.drawString(fitText(value, w - 12, 4), x + w - 6, y + h - 6, 4);
}

void infoLine(int y, const char* title, const String& value, uint16_t color = DisplayConfig::Text) {
  label(8, y, title, 2);
  frame.setTextDatum(TL_DATUM);
  frame.setTextColor(color);
  frame.drawString(fitText(value, Width - 120, 2), 112, y, 2);
}

void rpmBar(int x, int y, int w, const ValueView& rpm) {
  frame.fillRect(x, y, w, 6, DisplayConfig::Background);
  if (!rpm.hasValue()) return;
  const float ratio = constrain(rpm.value / static_cast<float>(DisplayConfig::RpmMax), 0.0f, 1.0f);
  const int warnX = x + static_cast<int>(w * DisplayConfig::RpmWarn / static_cast<float>(DisplayConfig::RpmMax));
  const int fill = static_cast<int>(w * ratio);
  frame.fillRect(x, y, fill, 6, colorFor(Metric::Rpm, rpm) == DisplayConfig::Text ? DisplayConfig::Accent
                                                                                    : colorFor(Metric::Rpm, rpm));
  frame.drawFastVLine(warnX, y - 2, 10, DisplayConfig::Warn);
}

String statusOr(const char* key, const char* fallback) {
  const String value = DisplayData::displayText(key);
  return value == "--" ? String(fallback) : value;
}

String ageText(uint32_t sinceMs) {
  if (sinceMs == 0) return "--";
  const uint32_t age = millis() - sinceMs;
  return age < 10000 ? String(age / 1000.0f, 1) + " s" : String(age / 1000) + " s";
}

// ---------------------------------------------------------------------------
// Pages
// ---------------------------------------------------------------------------

void drawStatusBar() {
  using namespace DisplayData;
  const bool connected = isEspNowConnected();
  const uint16_t background = connected ? DisplayConfig::Panel : DisplayConfig::Error;
  frame.fillRect(0, 0, Width, StatusBarHeight, background);
  frame.setTextColor(DisplayConfig::Text);
  frame.setTextDatum(ML_DATUM);
  String left = connected ? "Verbunden" : (runtime().lastReceivedAt == 0 ? "Warte auf Sender" : "Verbindung verloren");
  if (!connected && runtime().lastReceivedAt > 0) left += " " + ageText(runtime().lastReceivedAt);
  frame.drawString(left, 6, StatusBarHeight / 2, 2);

  if (live().simulated()) {
    frame.setTextDatum(MC_DATUM);
    frame.setTextColor(DisplayConfig::Warn);
    frame.drawString("SIMULATION", Width / 2 + 20, StatusBarHeight / 2, 2);
  }

  String right = String(currentPage + 1) + "/" + String(visiblePageCount());
  if (runtime().diagnosticPagesEnabled) right = "DIAG " + right;
  frame.setTextColor(DisplayConfig::Text);
  frame.setTextDatum(MR_DATUM);
  frame.drawString(right, Width - 6, StatusBarHeight / 2, 2);
}

void drawDrivePage() {
  const uint32_t now = millis();
  ValueView speed = valueOf(Metric::Speed);
  ValueView rpm = valueOf(Metric::Rpm);
  // Smooth only fresh values; stale values are shown as they are.
  if (speed.state == ValueState::Fresh) speed.value = speedSmoother.update(speed.value, now); else speedSmoother.reset();
  if (rpm.state == ValueState::Fresh) rpm.value = rpmSmoother.update(rpm.value, now); else rpmSmoother.reset();

  metricCard(4, 22, 154, 92, Metric::Speed, true, &speed);
  metricCard(162, 22, 154, 92, Metric::Rpm, true, &rpm);
  rpmBar(172, 44, 134, rpm);

  metricCard(4, 118, 77, 50, Metric::CoolantTemp, false);
  metricCard(83, 118, 77, 50, Metric::BatteryVoltage, false);
  metricCard(162, 118, 77, 50, Metric::OilTemp, false);

  const DisplayTelemetryValue* dtc = DisplayData::findValue("DTC");
  const bool dtcKnown = DisplayData::isStatusFresh(dtc);
  const bool fault = dtcKnown && dtc->status == "WARN";
  textCell(241, 118, 75, 50, "Fehler", fault ? "DTC!" : (dtcKnown ? "keine" : "--"),
           fault ? DisplayConfig::Warn : (dtcKnown ? DisplayConfig::Ok : DisplayConfig::Muted));
}

void drawEnginePage() {
  metricCard(4, 22, 154, 50, Metric::OilTemp, false);
  metricCard(162, 22, 154, 50, Metric::CoolantTemp, false);
  metricCard(4, 74, 154, 50, Metric::EngineLoad, false);
  metricCard(162, 74, 154, 50, Metric::IntakeTemp, false);
  metricCard(4, 126, 102, 42, Metric::BoostPressure, false);
  metricCard(109, 126, 102, 42, Metric::ManifoldPressure, false);
  metricCard(214, 126, 102, 42, Metric::Throttle, false);
}

void drawConsumptionPage() {
  metricCard(4, 22, 154, 50, Metric::InstantConsumption, false);
  metricCard(162, 22, 154, 50, Metric::AverageConsumption, false);
  metricCard(4, 74, 154, 50, Metric::FuelRate, false);
  metricCard(162, 74, 154, 50, Metric::Speed, false);
  metricCard(4, 126, 102, 42, Metric::FuelLevel, false);
  metricCard(109, 126, 102, 42, Metric::RunTime, false);
  metricCard(214, 126, 102, 42, Metric::AmbientTemp, false);
}

void drawLinkPage() {
  using namespace DisplayData;
  const auto& rt = runtime();
  const uint32_t total = rt.receivedPackets + rt.droppedPackets;
  const uint32_t quality = total == 0 ? 0 : (rt.receivedPackets * 100UL) / total;
  const bool connected = isEspNowConnected();
  frame.fillRoundRect(4, 22, Width - 8, Height - 26, 6, DisplayConfig::Panel);
  infoLine(28, "ESP-NOW", connected ? "verbunden" : "keine Daten", connected ? DisplayConfig::Ok : DisplayConfig::Error);
  infoLine(46, "Letztes Paket", ageText(rt.lastReceivedAt));
  infoLine(64, "Pakete", String(rt.receivedPackets) + " ok / " + String(rt.droppedPackets) + " verloren / " +
                             String(rt.crcErrors) + " CRC  (" + String(quality) + "%)");
  infoLine(82, "Heartbeat", ageText(rt.lastHeartbeatAt));
  infoLine(100, "CAN / OBD", (isCanStatusRecent() ? statusOr("CAN", "?") : String("--")) + " / " +
                                 (isObdStatusRecent() ? statusOr("OBD", "?") : String("--")));
  infoLine(118, "Firmware", String(ProjectConfig::FirmwareVersion) + "  Prot. " + String(ProjectConfig::ProtocolVersion));
  infoLine(136, "Hinweis", rt.lastError.length() > 0 ? rt.lastError : String("--"), DisplayConfig::Muted);
}

void drawDiagnosticPage() {
  using namespace DisplayData;
  const DisplayTelemetryValue* dtc = findValue("DTC");
  const bool fault = isStatusFresh(dtc) && dtc->status == "WARN";
  frame.fillRoundRect(4, 22, Width - 8, Height - 26, 6, DisplayConfig::Panel);
  String vin = displayText("VIN");
  if (vin == "--") vin = displayText("UDS_VIN");
  infoLine(28, "VIN", vin);
  infoLine(46, "OBD-DTC", displayText("DTC"), fault ? DisplayConfig::Warn : DisplayConfig::Text);
  infoLine(64, "UDS-DTC", displayText("UDS_DTC"));
  infoLine(82, "UDS", displayText("UDS"));
  infoLine(100, "ECUs", displayText("ReachableEcus"));
  infoLine(118, "NRC", displayText("UDS_NRC"));
  infoLine(136, "Backoff", displayText("UdsBackoff"), DisplayConfig::Muted);
}

void drawCanSnifferPage() {
  using namespace DisplayData;
  frame.fillRoundRect(4, 22, Width - 8, Height - 26, 6, DisplayConfig::Panel);
  if (!CANOBD2_DEV_TOOLS) {
    infoLine(28, "CAN-Sniffer", "nur in Entwickler-Builds", DisplayConfig::Muted);
    return;
  }
  infoLine(28, "Sniffer", displayText("CanSniffer"));
  infoLine(46, "Baseline", displayText("CanBaseline"));
  infoLine(64, "Kandidaten", displayText("CanCandidates"));
  infoLine(82, "CAN-ID", displayText("CanCandidateId"));
  infoLine(100, "Letzter", displayText("LastCAN"));
  infoLine(118, "Hinweis", displayText("CANHint"), DisplayConfig::Accent);
}

void drawMorePage() {
  metricCard(4, 22, 154, 50, Metric::Maf, false);
  metricCard(162, 22, 154, 50, Metric::ManifoldPressure, false);
  metricCard(4, 74, 154, 50, Metric::BarometricPressure, false);
  metricCard(162, 74, 154, 50, Metric::AmbientTemp, false);
  metricCard(4, 126, 154, 42, Metric::EcuVoltage, false);
  metricCard(162, 126, 154, 42, Metric::RunTime, false);
}

void drawPowerPage() {
  using namespace DisplayData;
  const auto& rt = runtime();
  frame.fillRoundRect(4, 22, Width - 8, Height - 26, 6, DisplayConfig::Panel);
  infoLine(28, "Fahrzeug", rt.vehicleState);
  infoLine(46, "Aktivitaet", String(rt.activityScore));
  infoLine(64, "Kommando", rt.powerCommand, rt.powerCommand == "Sleep" ? DisplayConfig::Warn : DisplayConfig::Text);
  infoLine(82, "Display", rt.displaySleepRequested ? "Schlaf" : "aktiv");
  infoLine(100, "Letzter CAN", ageText(rt.lastCanStatusAt));
  infoLine(118, "Letzter OBD", ageText(rt.lastObdStatusAt));
  infoLine(136, "Power-Status", ageText(rt.lastPowerStatusAt), DisplayConfig::Muted);
}


} // namespace

namespace DisplayRenderer {
void render() {
  frame.fillSprite(DisplayConfig::Background);
  drawStatusBar();
  switch (DisplayData::currentPage) {
    case 0: drawDrivePage(); break;
    case 1: drawEnginePage(); break;
    case 2: drawConsumptionPage(); break;
    case 3: drawLinkPage(); break;
    case 4: drawDiagnosticPage(); break;
    case 5: drawCanSnifferPage(); break;
    case 6: drawMorePage(); break;
    default: drawPowerPage(); break;
  }
  frame.pushSprite(0, 0);
}


} // namespace DisplayRenderer
