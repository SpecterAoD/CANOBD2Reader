#include "PID_Converter.h"
#include "PidDecoder.h"

PIDResult convertPID(byte pid, const byte* data, byte length) {
    const Obd::PidValue decoded = Obd::decodePid(pid, data, length);
    if (!decoded.valid) return {-1.0f, "INVALID"};
    return {decoded.value, decoded.unit};
}

const char* getPIDName(byte pid) {
    return Obd::pidName(pid);
}
