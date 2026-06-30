#include "PidDecoder.h"

namespace Obd {

PidValue decodePid(uint8_t pid, const uint8_t* data, std::size_t length) {
    if (data == nullptr) return {};

    switch (pid) {
        case 0x04:
            if (length < 1) return {};
            return {true, (data[0] * 100.0f) / 255.0f, "%"};
        case 0x05:
            if (length < 1) return {};
            return {true, data[0] - 40.0f, "C"};
        case 0x0B:
            if (length < 1) return {};
            return {true, static_cast<float>(data[0]), "kPa"};
        case 0x0C:
            if (length < 2) return {};
            return {true, ((data[0] * 256.0f) + data[1]) / 4.0f, "rpm"};
        case 0x0D:
            if (length < 1) return {};
            return {true, static_cast<float>(data[0]), "km/h"};
        case 0x0F:
            if (length < 1) return {};
            return {true, data[0] - 40.0f, "C"};
        case 0x10:
            if (length < 2) return {};
            return {true, ((data[0] * 256.0f) + data[1]) / 100.0f, "g/s"};
        case 0x11:
            if (length < 1) return {};
            return {true, (data[0] * 100.0f) / 255.0f, "%"};
        case 0x1F:
            if (length < 2) return {};
            return {true, (data[0] * 256.0f) + data[1], "s"};
        case 0x2F:
            if (length < 1) return {};
            return {true, (data[0] * 100.0f) / 255.0f, "%"};
        case 0x33:
            if (length < 1) return {};
            return {true, static_cast<float>(data[0]), "kPa"};
        case 0x42:
            if (length < 2) return {};
            return {true, ((data[0] * 256.0f) + data[1]) / 1000.0f, "V"};
        case 0x46:
            if (length < 1) return {};
            return {true, data[0] - 40.0f, "C"};
        case 0x5C:
            if (length < 1) return {};
            return {true, data[0] - 40.0f, "C"};
        case 0x5E:
            if (length < 2) return {};
            return {true, ((data[0] * 256.0f) + data[1]) / 20.0f, "L/h"};
        default:
            return {};
    }
}

const char* pidName(uint8_t pid) {
    switch (pid) {
        case 0x04: return "EngineLoad";
        case 0x05: return "CoolantTemp";
        case 0x0B: return "ManifoldAbsolutePressure";
        case 0x0C: return "RPM";
        case 0x0D: return "Speed";
        case 0x0F: return "IntakeTemp";
        case 0x10: return "MAF";
        case 0x11: return "Throttle";
        case 0x1F: return "RunTime";
        case 0x2F: return "FuelLevel";
        case 0x33: return "BarometricPressure";
        case 0x42: return "ControlVoltage";
        case 0x46: return "AmbientTemp";
        case 0x5C: return "OilTemp";
        case 0x5E: return "FuelRate";
        default: return "Unknown";
    }
}

}
