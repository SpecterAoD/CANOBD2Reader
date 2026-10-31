#pragma once

// Signed firmware images
// ======================
//
// Every OTA image (Web upload and GitHub update) must carry an Ed25519
// signature. The signing script (scripts/sign_firmware.py) appends a fixed-size
// trailer to the normal ESP32 application image:
//
//   offset  size  field
//   0       8     "CANOSIG1"            start magic
//   8       4     image length (LE)     bytes covered by the signature
//   12      4     key id (LE)           first 4 bytes of SHA-512(public key)
//   16      64    Ed25519 signature     over Domain || SHA-512(image)
//   80      8     "CANOSEND"            end magic
//
// The ESP32 bootloader ignores bytes after the end of the application image,
// so the trailer can be flashed together with the image. The verifier hashes
// the stream while it is written to the OTA partition and holds back the last
// TrailerSize bytes, because the trailer position is only known once the
// stream ends. Update.end(true) may only be called when finish() returns Ok.
//
// The private key never leaves the maintainer (GitHub Actions secret
// FIRMWARE_SIGNING_KEY); only public keys are compiled into the firmware.

#include <cstddef>
#include <cstdint>

#include "monocypher-ed25519.h"

namespace FirmwareSigning {

constexpr std::size_t TrailerSize = 88;
constexpr std::size_t PublicKeySize = 32;
constexpr std::size_t SignatureSize = 64;
constexpr std::size_t DigestSize = 64;

// Domain separation: a signature over this project's firmware can never be
// confused with an Ed25519 signature the same key might produce elsewhere.
constexpr char Domain[] = "CANOBD2-FIRMWARE-SIGNATURE-V1";
constexpr std::size_t DomainSize = sizeof(Domain) - 1;
constexpr std::size_t SignedMessageSize = DomainSize + DigestSize;

struct PublicKey {
    uint32_t keyId;
    uint8_t key[PublicKeySize];
};

enum class VerifyResult : uint8_t {
    Ok,
    NotStarted,
    MissingTrailer,
    LengthMismatch,
    UnknownKey,
    BadSignature
};

const char* resultName(VerifyResult result);

// Key id as used in the trailer: little-endian first 4 bytes of SHA-512(key).
uint32_t keyIdFor(const uint8_t key[PublicKeySize]);

// Builds Domain || digest. `out` must hold SignedMessageSize bytes.
void buildSignedMessage(const uint8_t digest[DigestSize], uint8_t* out);

class StreamingVerifier {
public:
    void begin();
    void update(const uint8_t* data, std::size_t length);

    // Validates trailer and signature against the given trusted keys. The
    // verifier is inactive afterwards; call begin() for the next image.
    VerifyResult finish(const PublicKey* keys, std::size_t keyCount);

    bool active() const { return active_; }
    // Bytes that are part of the image (everything except the held-back tail).
    std::size_t imageBytes() const { return hashedBytes_; }
    // Total bytes received including the trailer.
    std::size_t totalBytes() const { return hashedBytes_ + tailLength_; }
    // SHA-512 of the image, valid after finish() reached the signature check.
    const uint8_t* imageDigest() const { return digest_; }
    uint32_t trailerKeyId() const { return trailerKeyId_; }

private:
    void hash(const uint8_t* data, std::size_t length);

    crypto_sha512_ctx sha_{};
    uint8_t tail_[TrailerSize]{};
    std::size_t tailLength_ = 0;
    std::size_t hashedBytes_ = 0;
    uint8_t digest_[DigestSize]{};
    uint32_t trailerKeyId_ = 0;
    bool active_ = false;
};

} // namespace FirmwareSigning
