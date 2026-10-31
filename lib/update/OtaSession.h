#pragma once

// One OTA installation attempt (Web upload or GitHub download).
//
// Usage:
//   OtaSession session;
//   if (!session.begin()) ...           // erases nothing yet, opens partition
//   session.write(chunk, len) ...       // streams to flash + validator
//   if (session.finish()) restart();    // activates partition only if valid
//
// finish() calls Update.end(true) - which switches the boot partition - only
// after the signature and metadata checks passed. Any failure aborts the
// update and leaves the running firmware as boot partition.

#include <cstddef>
#include <cstdint>
#include <string>

#include "OtaImageValidator.h"

namespace FirmwareUpdate {

class OtaSession {
public:
    bool begin();
    bool write(const uint8_t* data, std::size_t length);
    bool finish();
    void abort(const char* reason);

    bool active() const { return active_; }
    const std::string& error() const { return error_; }
    // Version from the validated image metadata (empty until finish() succeeded).
    const std::string& installedVersion() const { return installedVersion_; }
    std::size_t receivedBytes() const { return validator_.receivedBytes(); }

private:
    void fail(const std::string& message);

    OtaImageValidator validator_;
    std::string error_;
    std::string installedVersion_;
    bool active_ = false;
};

} // namespace FirmwareUpdate
