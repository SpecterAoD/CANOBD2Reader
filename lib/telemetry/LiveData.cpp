#include "LiveData.h"

#include <cstring>
#include <cmath>

namespace LiveData {
namespace {

constexpr MetricInfo Table[MetricCount] = {
    {Metric::Rpm, "RPM", "Drehzahl", "rpm", 0, 0x0C},
    {Metric::Speed, "Speed", "Tempo", "km/h", 0, 0x0D},
    {Metric::CoolantTemp, "CoolantTemp", "Kuehlung", "C", 0, 0x05},
    {Metric::BatteryVoltage, "BatteryVoltage", "Batterie", "V", 1, -1},
    {Metric::EcuVoltage, "EcuVoltage", "ECU-Spannung", "V", 1, 0x42},
    {Metric::EngineLoad, "EngineLoad", "Last", "%", 0, 0x04},
    {Metric::IntakeTemp, "IntakeTemp", "Ansaugluft", "C", 0, 0x0F},
    {Metric::ManifoldPressure, "ManifoldAbsolutePressure", "Saugrohr", "kPa", 0, 0x0B},
    {Metric::BarometricPressure, "BarometricPressure", "Luftdruck", "kPa", 0, 0x33},
    {Metric::BoostPressure, "BoostPressureBar", "Ladedruck", "bar", 2, -1},
    {Metric::Maf, "MAF", "Luftmasse", "g/s", 1, 0x10},
    {Metric::Throttle, "Throttle", "Drossel", "%", 0, 0x11},
    {Metric::FuelLevel, "FuelLevel", "Tank", "%", 0, 0x2F},
    {Metric::RunTime, "RunTime", "Laufzeit", "s", 0, 0x1F},
    {Metric::AmbientTemp, "AmbientTemp", "Aussen", "C", 0, 0x46},
    {Metric::OilTemp, "OilTemp", "Oel", "C", 0, 0x5C},
    {Metric::FuelRate, "FuelRate", "Verbrauch", "L/h", 1, 0x5E},
    {Metric::InstantConsumption, "InstantConsumption", "Momentan", "L/100", 1, -1},
    {Metric::AverageConsumption, "AverageConsumption", "Schnitt", "L/100", 1, -1},
};

void writeLe32(uint8_t* out, uint32_t value) {
    out[0] = static_cast<uint8_t>(value);
    out[1] = static_cast<uint8_t>(value >> 8U);
    out[2] = static_cast<uint8_t>(value >> 16U);
    out[3] = static_cast<uint8_t>(value >> 24U);
}

uint32_t readLe32(const uint8_t* in) {
    return static_cast<uint32_t>(in[0]) | (static_cast<uint32_t>(in[1]) << 8U) |
           (static_cast<uint32_t>(in[2]) << 16U) | (static_cast<uint32_t>(in[3]) << 24U);
}

static_assert(sizeof(float) == 4, "IEEE-754 single precision expected");

} // namespace

const MetricInfo& info(Metric metric) {
    const std::size_t index = static_cast<std::size_t>(metric);
    return Table[index < MetricCount ? index : 0];
}

bool metricForPid(uint8_t pid, Metric& out) {
    for (const MetricInfo& entry : Table) {
        if (entry.obdPid == pid) {
            out = entry.metric;
            return true;
        }
    }
    return false;
}

bool metricForName(const char* name, Metric& out) {
    if (name == nullptr) return false;
    for (const MetricInfo& entry : Table) {
        if (std::strcmp(entry.name, name) == 0) {
            out = entry.metric;
            return true;
        }
    }
    return false;
}

std::size_t encodeSnapshot(const Snapshot& snapshot, uint8_t* out, std::size_t capacity) {
    if (out == nullptr || capacity < EncodedSnapshotSize) return 0;
    std::size_t pos = 0;
    out[pos++] = static_cast<uint8_t>(MetricCount);
    out[pos++] = snapshot.flags;
    writeLe32(out + pos, snapshot.validMask);
    pos += 4;
    writeLe32(out + pos, snapshot.unsupportedMask);
    pos += 4;
    for (float value : snapshot.values) {
        uint32_t bits = 0;
        std::memcpy(&bits, &value, sizeof(bits));
        writeLe32(out + pos, bits);
        pos += 4;
    }
    std::memcpy(out + pos, snapshot.ageTenths, MetricCount);
    pos += MetricCount;
    return pos;
}

bool decodeSnapshot(const uint8_t* data, std::size_t length, Snapshot& out) {
    if (data == nullptr || length != EncodedSnapshotSize || data[0] != MetricCount) return false;
    std::size_t pos = 1;
    out.flags = data[pos++];
    out.validMask = readLe32(data + pos);
    pos += 4;
    out.unsupportedMask = readLe32(data + pos);
    pos += 4;
    for (float& value : out.values) {
        const uint32_t bits = readLe32(data + pos);
        std::memcpy(&value, &bits, sizeof(value));
        pos += 4;
    }
    std::memcpy(out.ageTenths, data + pos, MetricCount);
    // Never trust bits beyond the known table.
    const uint32_t known = MetricCount >= 32 ? 0xFFFFFFFFu : ((1u << MetricCount) - 1u);
    out.validMask &= known;
    out.unsupportedMask &= known;
    return true;
}

void LiveValueTable::reset() {
    *this = LiveValueTable{};
}

void LiveValueTable::update(Metric metric, float value, uint32_t nowMs) {
    const uint8_t index = static_cast<uint8_t>(metric);
    if (index >= MetricCount || !std::isfinite(value)) return;
    values_[index] = value;
    updatedAt_[index] = nowMs;
    validMask_ |= 1u << index;
    unsupportedMask_ &= ~(1u << index);
}

void LiveValueTable::markUnsupported(Metric metric) {
    const uint8_t index = static_cast<uint8_t>(metric);
    if (index >= MetricCount) return;
    unsupportedMask_ |= 1u << index;
    validMask_ &= ~(1u << index);
}

void LiveValueTable::clear(Metric metric) {
    const uint8_t index = static_cast<uint8_t>(metric);
    if (index >= MetricCount) return;
    validMask_ &= ~(1u << index);
}

bool LiveValueTable::has(Metric metric) const {
    return (validMask_ >> static_cast<uint8_t>(metric)) & 1U;
}

float LiveValueTable::value(Metric metric) const {
    return values_[static_cast<uint8_t>(metric)];
}

Snapshot LiveValueTable::snapshot(uint32_t nowMs) const {
    Snapshot snap;
    snap.flags = simulated_ ? FlagSimulated : 0;
    snap.validMask = validMask_;
    snap.unsupportedMask = unsupportedMask_;
    for (std::size_t index = 0; index < MetricCount; ++index) {
        snap.values[index] = values_[index];
        if (!((validMask_ >> index) & 1U) || nowMs - updatedAt_[index] > MaxValueAgeMs) {
            snap.validMask &= ~(1u << index);
            snap.ageTenths[index] = AgeUnknown;
            continue;
        }
        const uint32_t ageTenths = (nowMs - updatedAt_[index]) / 100U;
        snap.ageTenths[index] = static_cast<uint8_t>(ageTenths >= AgeUnknown ? AgeUnknown - 1 : ageTenths);
    }
    return snap;
}

} // namespace LiveData
