// Binary live snapshot (telemetry protocol v3).
#include <unity.h>

#include "LiveData.h"
#include "TelemetryCodec.h"

using LiveData::Metric;

void setUp() {}
void tearDown() {}

void test_snapshot_fits_into_one_espnow_packet() {
    TEST_ASSERT_TRUE(LiveData::EncodedSnapshotSize <= ProjectConfig::TelemetryPayloadSize);
}

void test_snapshot_roundtrip_keeps_values_masks_and_ages() {
    LiveData::LiveValueTable table;
    table.update(Metric::Rpm, 2450.0f, 1000);
    table.update(Metric::Speed, 87.5f, 1700);
    table.markUnsupported(Metric::OilTemp);
    table.setSimulated(true);

    const LiveData::Snapshot snap = table.snapshot(2000);
    uint8_t buffer[LiveData::EncodedSnapshotSize];
    TEST_ASSERT_EQUAL(LiveData::EncodedSnapshotSize, LiveData::encodeSnapshot(snap, buffer, sizeof(buffer)));

    LiveData::Snapshot decoded;
    TEST_ASSERT_TRUE(LiveData::decodeSnapshot(buffer, sizeof(buffer), decoded));
    TEST_ASSERT_TRUE(decoded.valid(Metric::Rpm));
    TEST_ASSERT_TRUE(decoded.valid(Metric::Speed));
    TEST_ASSERT_FALSE(decoded.valid(Metric::CoolantTemp));
    TEST_ASSERT_TRUE(decoded.unsupported(Metric::OilTemp));
    TEST_ASSERT_EQUAL_FLOAT(2450.0f, decoded.value(Metric::Rpm));
    TEST_ASSERT_EQUAL_FLOAT(87.5f, decoded.value(Metric::Speed));
    TEST_ASSERT_EQUAL_UINT8(10, decoded.ageTenths[static_cast<int>(Metric::Rpm)]);
    TEST_ASSERT_EQUAL_UINT8(3, decoded.ageTenths[static_cast<int>(Metric::Speed)]);
    TEST_ASSERT_EQUAL_UINT8(LiveData::AgeUnknown, decoded.ageTenths[static_cast<int>(Metric::CoolantTemp)]);
    TEST_ASSERT_EQUAL_UINT8(LiveData::FlagSimulated, decoded.flags);
}

void test_very_old_values_expire_in_snapshot() {
    LiveData::LiveValueTable table;
    table.update(Metric::FuelLevel, 50.0f, 0);
    const LiveData::Snapshot snap = table.snapshot(600000);
    TEST_ASSERT_FALSE(snap.valid(Metric::FuelLevel));
    TEST_ASSERT_EQUAL_UINT8(LiveData::AgeUnknown, snap.ageTenths[static_cast<int>(Metric::FuelLevel)]);
}

void test_new_value_clears_unsupported_flag() {
    LiveData::LiveValueTable table;
    table.markUnsupported(Metric::Maf);
    table.update(Metric::Maf, 12.0f, 10);
    const LiveData::Snapshot snap = table.snapshot(10);
    TEST_ASSERT_TRUE(snap.valid(Metric::Maf));
    TEST_ASSERT_FALSE(snap.unsupported(Metric::Maf));
}

void test_decode_rejects_wrong_size_or_table_version() {
    uint8_t buffer[LiveData::EncodedSnapshotSize] = {};
    LiveData::Snapshot snap;
    TEST_ASSERT_FALSE(LiveData::decodeSnapshot(buffer, sizeof(buffer) - 1, snap));
    buffer[0] = static_cast<uint8_t>(LiveData::MetricCount + 1);
    TEST_ASSERT_FALSE(LiveData::decodeSnapshot(buffer, sizeof(buffer), snap));
}

void test_pid_and_name_lookup() {
    Metric metric = Metric::Count;
    TEST_ASSERT_TRUE(LiveData::metricForPid(0x0C, metric));
    TEST_ASSERT_EQUAL(static_cast<int>(Metric::Rpm), static_cast<int>(metric));
    TEST_ASSERT_TRUE(LiveData::metricForName("BoostPressureBar", metric));
    TEST_ASSERT_EQUAL(static_cast<int>(Metric::BoostPressure), static_cast<int>(metric));
    TEST_ASSERT_FALSE(LiveData::metricForPid(0x01, metric));
    for (std::size_t index = 0; index < LiveData::MetricCount; ++index) {
        TEST_ASSERT_EQUAL(static_cast<int>(index), static_cast<int>(LiveData::info(static_cast<Metric>(index)).metric));
    }
}

void test_snapshot_travels_in_telemetry_packet() {
    LiveData::LiveValueTable table;
    table.update(Metric::CoolantTemp, 91.0f, 0);
    uint8_t buffer[LiveData::EncodedSnapshotSize];
    const std::size_t size = LiveData::encodeSnapshot(table.snapshot(0), buffer, sizeof(buffer));

    Telemetry::TelemetryPacket packet;
    TEST_ASSERT_TRUE(Telemetry::TelemetryCodec::encode(packet, Telemetry::PacketType::LiveSnapshot, 7, 0, buffer, size));
    Telemetry::TelemetryPacket received;
    TEST_ASSERT_EQUAL(static_cast<int>(Telemetry::DecodeStatus::Ok),
                      static_cast<int>(Telemetry::TelemetryCodec::decode(reinterpret_cast<const uint8_t*>(&packet),
                                                                         sizeof(packet), received)));
    LiveData::Snapshot snap;
    TEST_ASSERT_TRUE(LiveData::decodeSnapshot(received.payload, received.payloadLength, snap));
    TEST_ASSERT_EQUAL_FLOAT(91.0f, snap.value(Metric::CoolantTemp));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_snapshot_fits_into_one_espnow_packet);
    RUN_TEST(test_snapshot_roundtrip_keeps_values_masks_and_ages);
    RUN_TEST(test_very_old_values_expire_in_snapshot);
    RUN_TEST(test_new_value_clears_unsupported_flag);
    RUN_TEST(test_decode_rejects_wrong_size_or_table_version);
    RUN_TEST(test_pid_and_name_lookup);
    RUN_TEST(test_snapshot_travels_in_telemetry_packet);
    return UNITY_END();
}
