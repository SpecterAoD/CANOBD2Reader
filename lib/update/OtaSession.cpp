#include "OtaSession.h"

#include "common_config.h"
#include "config/ProjectConfig.h"
#include "config/UpdateConfig.h"
#include "config/FirmwareSigningKeys.h"

#if defined(ARDUINO)
  #include <Arduino.h>
  #include <Update.h>
#endif

namespace FirmwareUpdate {

#if defined(ARDUINO)
namespace {
std::string updateError(const char* prefix) {
    return std::string(prefix) + " (" + Update.errorString() + ")";
}
} // namespace
#endif

void OtaSession::fail(const std::string& message) {
    error_ = message;
#if defined(ARDUINO)
    if (Update.isRunning()) Update.abort();
#endif
    active_ = false;
}

bool OtaSession::begin() {
    error_.clear();
    installedVersion_.clear();
    validator_.begin();
#if defined(ARDUINO)
    if (Update.isRunning()) Update.abort();
    if (UpdateConfig::RejectWhenSketchSpaceUnknown && ESP.getFreeSketchSpace() == 0) {
        fail("Freier OTA-Speicher unbekannt");
        return false;
    }
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        fail(updateError("Update.begin fehlgeschlagen"));
        return false;
    }
#endif
    active_ = true;
    return true;
}

bool OtaSession::write(const uint8_t* data, std::size_t length) {
    if (!active_) return false;
    if (data == nullptr || length == 0) return true;

    validator_.update(data, length);
#if defined(ARDUINO)
    // Update.write() takes a non-const pointer but does not modify the data.
    if (Update.write(const_cast<uint8_t*>(data), length) != length) {
        fail(updateError("Flash-Schreibfehler"));
        return false;
    }
#endif
    return true;
}

bool OtaSession::finish() {
    if (!active_) {
        if (error_.empty()) error_ = "Kein aktives Update";
        return false;
    }

    const OtaImageValidator::Expectation expected{
        ProjectConfig::TargetName,
        ProjectConfig::ProtocolVersion,
        CANOBD2_OTA_METADATA_SCHEMA,
    };
    const ImageCheck check = validator_.finish(expected,
                                               FirmwareSigningKeys::Trusted,
                                               FirmwareSigningKeys::TrustedCount);
    if (check != ImageCheck::Ok) {
        std::string message = imageCheckText(check);
        if (check == ImageCheck::SignatureInvalid) {
            message += std::string(" (") + FirmwareSigning::resultName(validator_.signatureResult()) + ")";
        } else if (check == ImageCheck::WrongTarget) {
            message += " (" + validator_.metadata().target + ")";
        }
        fail(message);
        return false;
    }

#if defined(ARDUINO)
    // evenIfRemaining=true: the image size is only known at the end.
    if (!Update.end(true)) {
        fail(updateError("Update.end fehlgeschlagen"));
        return false;
    }
#endif
    installedVersion_ = validator_.metadata().version;
    active_ = false;
    return true;
}

void OtaSession::abort(const char* reason) {
    if (!active_) return;
    fail(reason == nullptr ? "Abgebrochen" : reason);
}

} // namespace FirmwareUpdate
