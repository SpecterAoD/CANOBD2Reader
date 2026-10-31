#pragma once

#include <cstddef>
#include <string>

namespace FirmwareUpdate {

class OtaSession;

// HTTPS access to the GitHub release assets (manifest + firmware images).
// TLS uses the pinned root bundle from config/UpdateConfig.h; redirects to the
// GitHub asset CDN are followed.
class GitHubUpdateClient {
public:
    static bool fetchText(const char* url, std::size_t maxBytes, std::string& body, std::string& error);
    // Streams the firmware image into an already started OtaSession.
    static bool downloadInto(const char* url, OtaSession& session, std::string& error);
};

} // namespace FirmwareUpdate
