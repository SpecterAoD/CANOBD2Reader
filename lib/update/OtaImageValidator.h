#pragma once

// Platform-independent validation of an incoming OTA image stream.
//
// The validator is fed the raw bytes exactly as they are written to the OTA
// partition. It combines two independent checks:
//   1. Ed25519 signature trailer (FirmwareSigning::StreamingVerifier) - proves
//      that the image was built and signed by the maintainer.
//   2. The embedded metadata record (lib/common/FirmwareMetadata.cpp)
//        CANOBD2_FW_METADATA_BEGIN;target=..;version=..;schema=..;protocol=..;...;CANOBD2_FW_METADATA_END
//      - proves that the (authentic) image is meant for this device type and
//      speaks the same telemetry protocol.
// Only when both pass may the caller activate the new partition.

#include <cstddef>
#include <cstdint>
#include <string>

#include "FirmwareSignature.h"

namespace FirmwareUpdate {

struct FirmwareMetadataRecord {
    bool found = false;
    std::string target;
    std::string version;
    int schema = 0;
    int protocol = 0;
};

// Finds and parses the first metadata record in a byte stream that arrives in
// arbitrary chunks. Records longer than MaxRecordSize are ignored.
class FirmwareMetadataScanner {
public:
    static constexpr std::size_t MaxRecordSize = 320;

    void reset();
    void update(const uint8_t* data, std::size_t length);
    const FirmwareMetadataRecord& record() const { return record_; }

private:
    void consume(char c);
    void parseRecord();

    FirmwareMetadataRecord record_;
    std::size_t beginMatched_ = 0;
    bool capturing_ = false;
    std::string buffer_;
};

enum class ImageCheck : uint8_t {
    Ok,
    SignatureInvalid,
    MetadataMissing,
    WrongTarget,
    WrongProtocol,
    WrongSchema
};

const char* imageCheckText(ImageCheck check);

class OtaImageValidator {
public:
    struct Expectation {
        const char* target;
        int protocol;
        int schema;
    };

    void begin();
    void update(const uint8_t* data, std::size_t length);
    ImageCheck finish(const Expectation& expected,
                      const FirmwareSigning::PublicKey* keys,
                      std::size_t keyCount);

    const FirmwareMetadataRecord& metadata() const { return scanner_.record(); }
    FirmwareSigning::VerifyResult signatureResult() const { return signatureResult_; }
    std::size_t receivedBytes() const { return verifier_.totalBytes(); }
    bool active() const { return verifier_.active(); }

private:
    FirmwareSigning::StreamingVerifier verifier_;
    FirmwareMetadataScanner scanner_;
    FirmwareSigning::VerifyResult signatureResult_ = FirmwareSigning::VerifyResult::NotStarted;
};

} // namespace FirmwareUpdate
