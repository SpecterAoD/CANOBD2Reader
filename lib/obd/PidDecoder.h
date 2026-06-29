#pragma once

#include <cstddef>
#include <cstdint>

namespace Obd {

struct PidValue {
    bool valid = false;
    float value = 0.0f;
    const char* unit = "";
};

enum Mode : uint8_t {
    CurrentData = 0x01,
    FreezeFrame = 0x02,
    StoredDtc = 0x03,
    VehicleInfo = 0x09
};

PidValue decodePid(uint8_t pid, const uint8_t* data, std::size_t length);
const char* pidName(uint8_t pid);

}
