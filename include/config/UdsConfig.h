#pragma once

#include <cstddef>
#include <cstdint>

namespace UdsConfig {

struct EcuTarget {
    uint8_t diagnosticAddress;
    uint32_t requestId;
    uint32_t responseId;
    const char* name;
    bool scanByDefault;
};

struct DidCandidate {
    uint16_t did;
    const char* name;
    bool scanByDefault;
};

constexpr bool EnableUdsWriteServices = false;
constexpr bool EnableSecurityAccess = false;
constexpr bool EnableCodingOrAdaptation = false;
constexpr bool EnableExtendedDiagnosticSessionForCapabilityScan = false;
constexpr uint8_t TesterPresentSubFunction = 0x00;
constexpr uint8_t TesterPresentSuppressPositiveResponseSubFunction = 0x80;

// VW/MQB diagnostic targets commonly reachable through the OBD-II diagnostic
// CAN gateway. Standard OBD Mode 01 still uses functional 0x7DF where useful;
// UDS Service 0x22/0x19 scans should use these physical targets.
inline constexpr EcuTarget EcuTargets[] = {
    {0x01, 0x7E0, 0x7E8, "Engine ECU", true},
    {0x02, 0x7E1, 0x7E9, "Transmission", true},
    {0x03, 0x7E2, 0x7EA, "ABS/ESP", true},
    {0x09, 0x7E4, 0x7EC, "Body Control Module", true},
    {0x08, 0x746, 0x7B0, "HVAC", true},
    {0x08, 0x7E5, 0x7ED, "HVAC legacy/alternate", false},
    {0x17, 0x714, 0x77E, "Instrument Cluster", true},
    // 0x7DF is the functional diagnostic broadcast. It is documented here for
    // OBD/diagnostic context, but not used by the physical UDS capability scan.
    {0x19, 0x7DF, 0x000, "Diagnostic Gateway Broadcast", false}
};

inline constexpr std::size_t EcuTargetCount = sizeof(EcuTargets) / sizeof(EcuTargets[0]);

// Safe read-only DID candidates. The F1xx identifiers are common UDS
// identification DIDs. VW-specific live-data DIDs are documented here as
// manual discovery candidates, but are not scanned by default because scaling
// and access permissions vary by ECU software.
inline constexpr DidCandidate DidCandidates[] = {
    {0xF180, "BootSoftwareIdentification", true},
    {0xF181, "ApplicationSoftwareIdentification", true},
    {0xF182, "ApplicationDataIdentification", true},
    {0xF186, "ActiveDiagnosticSession", true},
    {0xF187, "ManufacturerSparePartNumber", true},
    {0xF188, "EcuSoftwareNumber", true},
    {0xF189, "EcuSoftwareVersion", true},
    {0xF18A, "SystemSupplierIdentifier", true},
    {0xF18B, "EcuManufacturingDate", true},
    {0xF18C, "EcuSerialNumber", true},
    {0xF190, "VIN", true},
    {0xF195, "SoftwareVersion", true},
    {0x2001, "VW candidate Odometer", false},
    {0x1103, "VW candidate CoolantTemperature", false},
    {0x1004, "VW candidate CoolantTemperatureAlt", false},
    {0x1312, "VW candidate OilTemperature", false},
    {0x2024, "VW candidate OilTemperatureAlt", false},
    {0x1000, "VW candidate EngineSpeed", false},
    {0x1470, "VW candidate BoostPressureSpecified", false},
    {0x1471, "VW candidate BoostPressureActual", false},
    {0x1008, "VW candidate BatteryVoltage", false},
    {0x210D, "VW candidate BatteryVoltageAlt", false},
    {0x1700, "VW candidate ExhaustGasTemperatureGroup", false}
};

inline constexpr std::size_t DidCandidateCount = sizeof(DidCandidates) / sizeof(DidCandidates[0]);

inline constexpr const EcuTarget* findTargetByRequestId(uint32_t requestId) {
    for (std::size_t index = 0; index < EcuTargetCount; ++index) {
        if (EcuTargets[index].requestId == requestId) return &EcuTargets[index];
    }
    return nullptr;
}

inline constexpr uint32_t responseIdForRequestId(uint32_t requestId) {
    const EcuTarget* target = findTargetByRequestId(requestId);
    if (target != nullptr && target->responseId != 0) return target->responseId;
    return requestId + 8U;
}

inline constexpr bool isConfiguredRequestId(uint32_t requestId) {
    return findTargetByRequestId(requestId) != nullptr;
}

inline constexpr std::size_t DefaultScanTargetCount() {
    std::size_t count = 0;
    for (std::size_t index = 0; index < EcuTargetCount; ++index) {
        if (EcuTargets[index].scanByDefault) ++count;
    }
    return count;
}

inline constexpr std::size_t DefaultDidScanCount() {
    std::size_t count = 0;
    for (std::size_t index = 0; index < DidCandidateCount; ++index) {
        if (DidCandidates[index].scanByDefault) ++count;
    }
    return count;
}

} // namespace UdsConfig
