#include "FirmwareSignature.h"

#include <cstring>

namespace FirmwareSigning {
namespace {

constexpr char StartMagic[8] = {'C', 'A', 'N', 'O', 'S', 'I', 'G', '1'};
constexpr char EndMagic[8] = {'C', 'A', 'N', 'O', 'S', 'E', 'N', 'D'};
constexpr std::size_t LengthOffset = 8;
constexpr std::size_t KeyIdOffset = 12;
constexpr std::size_t SignatureOffset = 16;
constexpr std::size_t EndMagicOffset = SignatureOffset + SignatureSize;

static_assert(EndMagicOffset + sizeof(EndMagic) == TrailerSize, "trailer layout mismatch");

uint32_t readLe32(const uint8_t* bytes) {
    return static_cast<uint32_t>(bytes[0]) |
           (static_cast<uint32_t>(bytes[1]) << 8U) |
           (static_cast<uint32_t>(bytes[2]) << 16U) |
           (static_cast<uint32_t>(bytes[3]) << 24U);
}

} // namespace

const char* resultName(VerifyResult result) {
    switch (result) {
        case VerifyResult::Ok: return "ok";
        case VerifyResult::NotStarted: return "not_started";
        case VerifyResult::MissingTrailer: return "unsigned";
        case VerifyResult::LengthMismatch: return "length_mismatch";
        case VerifyResult::UnknownKey: return "unknown_key";
        case VerifyResult::BadSignature: return "bad_signature";
    }
    return "unknown";
}

uint32_t keyIdFor(const uint8_t key[PublicKeySize]) {
    uint8_t hash[DigestSize];
    crypto_sha512(hash, key, PublicKeySize);
    return readLe32(hash);
}

void buildSignedMessage(const uint8_t digest[DigestSize], uint8_t* out) {
    std::memcpy(out, Domain, DomainSize);
    std::memcpy(out + DomainSize, digest, DigestSize);
}

void StreamingVerifier::begin() {
    crypto_sha512_init(&sha_);
    tailLength_ = 0;
    hashedBytes_ = 0;
    trailerKeyId_ = 0;
    std::memset(digest_, 0, sizeof(digest_));
    active_ = true;
}

void StreamingVerifier::hash(const uint8_t* data, std::size_t length) {
    if (length == 0) return;
    crypto_sha512_update(&sha_, data, length);
    hashedBytes_ += length;
}

void StreamingVerifier::update(const uint8_t* data, std::size_t length) {
    if (!active_ || data == nullptr || length == 0) return;

    if (length >= TrailerSize) {
        // The old tail and everything except the last TrailerSize new bytes are
        // definitely image data.
        hash(tail_, tailLength_);
        hash(data, length - TrailerSize);
        std::memcpy(tail_, data + length - TrailerSize, TrailerSize);
        tailLength_ = TrailerSize;
        return;
    }

    // Short chunk: push the oldest tail bytes into the hash so that the tail
    // keeps holding exactly the newest TrailerSize bytes.
    const std::size_t total = tailLength_ + length;
    if (total > TrailerSize) {
        const std::size_t overflow = total - TrailerSize;
        hash(tail_, overflow);
        std::memmove(tail_, tail_ + overflow, tailLength_ - overflow);
        tailLength_ -= overflow;
    }
    std::memcpy(tail_ + tailLength_, data, length);
    tailLength_ += length;
}

VerifyResult StreamingVerifier::finish(const PublicKey* keys, std::size_t keyCount) {
    if (!active_) return VerifyResult::NotStarted;
    active_ = false;

    if (tailLength_ != TrailerSize ||
        std::memcmp(tail_, StartMagic, sizeof(StartMagic)) != 0 ||
        std::memcmp(tail_ + EndMagicOffset, EndMagic, sizeof(EndMagic)) != 0) {
        return VerifyResult::MissingTrailer;
    }

    if (readLe32(tail_ + LengthOffset) != hashedBytes_) return VerifyResult::LengthMismatch;

    trailerKeyId_ = readLe32(tail_ + KeyIdOffset);
    const PublicKey* trusted = nullptr;
    for (std::size_t index = 0; keys != nullptr && index < keyCount; ++index) {
        if (keys[index].keyId == trailerKeyId_) {
            trusted = &keys[index];
            break;
        }
    }
    if (trusted == nullptr) return VerifyResult::UnknownKey;

    crypto_sha512_final(&sha_, digest_);
    uint8_t message[SignedMessageSize];
    buildSignedMessage(digest_, message);
    const int check = crypto_ed25519_check(tail_ + SignatureOffset, trusted->key, message, sizeof(message));
    return check == 0 ? VerifyResult::Ok : VerifyResult::BadSignature;
}

} // namespace FirmwareSigning
