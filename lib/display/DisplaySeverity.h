#pragma once

#include <cstdint>

namespace DisplayLogic {

enum class DisplaySeverity : uint8_t {
    Ok,
    Warning,
    Critical,
    Timeout,
    Unknown
};

DisplaySeverity severityForMetric(const char* metricName,
                                  float value,
                                  bool fresh,
                                  const char* status = "OK");

const char* severityName(DisplaySeverity severity);

}
