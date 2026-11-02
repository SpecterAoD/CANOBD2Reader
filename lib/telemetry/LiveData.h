#pragma once

// Live vehicle values (telemetry protocol v3)
// ===========================================
//
// The sender keeps the latest value of every metric in a LiveValueTable and
// broadcasts the complete table as one binary snapshot packet
// (PacketType::LiveSnapshot) every SnapshotIntervalMs. The display therefore
// always has a consistent view of all values, regardless of which PID was
// polled last, and no value "falls out" just because its update is slower.
//
// Wire format (little endian, inside TelemetryPacket::payload):
//   uint8  metricCount            == MetricCount (versioning of the table)
//   uint8  flags                  bit0: values are simulated
//   uint32 validMask              bit n: metric n has a value
//   uint32 unsupportedMask        bit n: ECU does not support metric n
//   float  values[MetricCount]
//   uint8  ageTenths[MetricCount] age of each value in 0.1 s, 254 = saturated age, 255 = missing/expired
//
// Status/diagnostic information (DTC, VIN, UDS, CAN hints, power state) stays
// in the text packets, it is rare and not time critical.

#include <cstddef>
#include <cstdint>

namespace LiveData {

enum class Metric : uint8_t {
    Rpm,
    Speed,
    CoolantTemp,
    BatteryVoltage,
    EcuVoltage,
    EngineLoad,
    IntakeTemp,
    ManifoldPressure,
    BarometricPressure,
    BoostPressure,
    Maf,
    Throttle,
    FuelLevel,
    RunTime,
    AmbientTemp,
    OilTemp,
    FuelRate,
    InstantConsumption,
    AverageConsumption,
    Count
};

constexpr std::size_t MetricCount = static_cast<std::size_t>(Metric::Count);
static_assert(MetricCount <= 32, "masks are 32 bit");

struct MetricInfo {
    Metric metric;
    const char* name;   // stable identifier, also used by the simulation
    const char* label;  // short display label (ASCII)
    const char* unit;
    uint8_t decimals;
    int16_t obdPid;     // Mode 01 PID, -1 if derived
};

const MetricInfo& info(Metric metric);
// Lookup by Mode 01 PID or by name; returns false if unknown.
bool metricForPid(uint8_t pid, Metric& out);
bool metricForName(const char* name, Metric& out);

constexpr uint8_t AgeUnknown = 255;
constexpr uint32_t MaxValueAgeMs = 60000;
constexpr uint8_t FlagSimulated = 0x01;

struct Snapshot {
    uint8_t flags = 0;
    uint32_t validMask = 0;
    uint32_t unsupportedMask = 0;
    float values[MetricCount]{};
    uint8_t ageTenths[MetricCount]{};

    bool valid(Metric m) const { return (validMask >> static_cast<uint8_t>(m)) & 1U; }
    bool unsupported(Metric m) const { return (unsupportedMask >> static_cast<uint8_t>(m)) & 1U; }
    float value(Metric m) const { return values[static_cast<uint8_t>(m)]; }
};

constexpr std::size_t EncodedSnapshotSize = 2 + 4 + 4 + MetricCount * 4 + MetricCount;

// Returns bytes written or 0 if `capacity` is too small.
std::size_t encodeSnapshot(const Snapshot& snapshot, uint8_t* out, std::size_t capacity);
bool decodeSnapshot(const uint8_t* data, std::size_t length, Snapshot& out);

// Sender-side store of the latest values.
class LiveValueTable {
public:
    void reset();
    void update(Metric metric, float value, uint32_t nowMs);
    void markUnsupported(Metric metric);
    // Value no longer meaningful (e.g. L/100 km while standing): shown as "--".
    void clear(Metric metric);
    void setSimulated(bool simulated) { simulated_ = simulated; }
    bool has(Metric metric) const;
    float value(Metric metric) const;
    Snapshot snapshot(uint32_t nowMs) const;

private:
    float values_[MetricCount]{};
    uint32_t updatedAt_[MetricCount]{};
    uint32_t validMask_ = 0;
    uint32_t unsupportedMask_ = 0;
    bool simulated_ = false;
};

} // namespace LiveData
