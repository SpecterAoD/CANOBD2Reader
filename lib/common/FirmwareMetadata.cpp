#include "common_config.h"

// This marker is deliberately kept as a plain ASCII string in the firmware
// image. Web-OTA can validate the uploaded binary without parsing ESP image
// internals and without comparing against the currently installed version.
#if defined(__GNUC__)
#define CANOBD2_METADATA_USED __attribute__((used))
#define CANOBD2_METADATA_SECTION __attribute__((section(".rodata.canobd2_metadata")))
#else
#define CANOBD2_METADATA_USED
#define CANOBD2_METADATA_SECTION
#endif

extern "C" CANOBD2_METADATA_USED CANOBD2_METADATA_SECTION const char CANOBD2_FIRMWARE_METADATA[] =
    "CANOBD2_FW_METADATA_BEGIN;"
    "target=" CANOBD2_TARGET_NAME ";"
    "version=" CANOBD2_FIRMWARE_VERSION ";"
    "protocol=" CANOBD2_STRINGIFY(CANOBD2_PROTOCOL_VERSION) ";"
    "compat_versions=" CANOBD2_OTA_COMPAT_VERSION_MARKERS ";"
    "CANOBD2_FW_METADATA_END";

namespace FirmwareMetadata {

const char* text() {
    return CANOBD2_FIRMWARE_METADATA;
}

}
