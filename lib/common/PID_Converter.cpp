#include "PID_Converter.h"
#include "PidDecoder.h"

PIDResult convertPID(uint8_t pid, const uint8_t* data, uint8_t length) {
    return Obd::decodePid(pid, data, length);
}

const char* getPIDName(uint8_t pid) {
    return Obd::pidName(pid);
}
