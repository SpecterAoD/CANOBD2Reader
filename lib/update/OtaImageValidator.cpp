#include "OtaImageValidator.h"

#include <cstdlib>
#include <cstring>

namespace FirmwareUpdate {
namespace {

constexpr char BeginMarker[] = "CANOBD2_FW_METADATA_BEGIN;";
constexpr std::size_t BeginMarkerLength = sizeof(BeginMarker) - 1;
constexpr char EndMarker[] = "CANOBD2_FW_METADATA_END";
constexpr std::size_t EndMarkerLength = sizeof(EndMarker) - 1;

bool endsWith(const std::string& text, const char* suffix, std::size_t suffixLength) {
    return text.size() >= suffixLength &&
           text.compare(text.size() - suffixLength, suffixLength, suffix) == 0;
}

} // namespace

void FirmwareMetadataScanner::reset() {
    record_ = {};
    beginMatched_ = 0;
    capturing_ = false;
    buffer_.clear();
}

void FirmwareMetadataScanner::update(const uint8_t* data, std::size_t length) {
    if (data == nullptr || record_.found) return;
    for (std::size_t index = 0; index < length && !record_.found; ++index) {
        consume(static_cast<char>(data[index]));
    }
}

void FirmwareMetadataScanner::consume(char c) {
    if (!capturing_) {
        // Incremental match of the begin marker. The marker has no repeated
        // prefix, so restarting at 0 (or 1 on 'C') is a correct KMP fallback.
        if (c == BeginMarker[beginMatched_]) {
            ++beginMatched_;
            if (beginMatched_ == BeginMarkerLength) {
                capturing_ = true;
                beginMatched_ = 0;
                buffer_.clear();
            }
        } else {
            beginMatched_ = (c == BeginMarker[0]) ? 1 : 0;
        }
        return;
    }

    // Metadata is printable ASCII; a NUL or binary byte means this was a stray
    // copy of the marker (e.g. inside a log string), so keep searching.
    if (c == '\0' || static_cast<unsigned char>(c) < 0x20 || buffer_.size() >= MaxRecordSize) {
        capturing_ = false;
        buffer_.clear();
        beginMatched_ = (c == BeginMarker[0]) ? 1 : 0;
        return;
    }

    buffer_.push_back(c);
    if (endsWith(buffer_, EndMarker, EndMarkerLength)) {
        buffer_.resize(buffer_.size() - EndMarkerLength);
        parseRecord();
        capturing_ = false;
    }
}

void FirmwareMetadataScanner::parseRecord() {
    FirmwareMetadataRecord parsed;
    std::size_t start = 0;
    while (start < buffer_.size()) {
        std::size_t end = buffer_.find(';', start);
        if (end == std::string::npos) end = buffer_.size();
        const std::string field = buffer_.substr(start, end - start);
        const std::size_t equals = field.find('=');
        if (equals != std::string::npos) {
            const std::string key = field.substr(0, equals);
            const std::string value = field.substr(equals + 1);
            if (key == "target") parsed.target = value;
            else if (key == "version") parsed.version = value;
            else if (key == "schema") parsed.schema = std::atoi(value.c_str());
            else if (key == "protocol") parsed.protocol = std::atoi(value.c_str());
        }
        start = end + 1;
    }
    parsed.found = !parsed.target.empty() && !parsed.version.empty();
    if (parsed.found) record_ = parsed;
}

const char* imageCheckText(ImageCheck check) {
    switch (check) {
        case ImageCheck::Ok: return "ok";
        case ImageCheck::SignatureInvalid: return "Signatur ungueltig oder fehlt";
        case ImageCheck::MetadataMissing: return "Firmware-Metadaten fehlen";
        case ImageCheck::WrongTarget: return "Firmware fuer anderes Geraet";
        case ImageCheck::WrongProtocol: return "Telemetrie-Protokoll inkompatibel";
        case ImageCheck::WrongSchema: return "Metadaten-Schema unbekannt";
    }
    return "unbekannt";
}

void OtaImageValidator::begin() {
    verifier_.begin();
    scanner_.reset();
    signatureResult_ = FirmwareSigning::VerifyResult::NotStarted;
}

void OtaImageValidator::update(const uint8_t* data, std::size_t length) {
    verifier_.update(data, length);
    scanner_.update(data, length);
}

ImageCheck OtaImageValidator::finish(const Expectation& expected,
                                     const FirmwareSigning::PublicKey* keys,
                                     std::size_t keyCount) {
    // Signature first: metadata of an unauthenticated image is meaningless.
    signatureResult_ = verifier_.finish(keys, keyCount);
    if (signatureResult_ != FirmwareSigning::VerifyResult::Ok) return ImageCheck::SignatureInvalid;

    const FirmwareMetadataRecord& meta = scanner_.record();
    if (!meta.found) return ImageCheck::MetadataMissing;
    if (expected.target == nullptr || meta.target != expected.target) return ImageCheck::WrongTarget;
    if (meta.schema != expected.schema) return ImageCheck::WrongSchema;
    if (meta.protocol != expected.protocol) return ImageCheck::WrongProtocol;
    return ImageCheck::Ok;
}

} // namespace FirmwareUpdate
