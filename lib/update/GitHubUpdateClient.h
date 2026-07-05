#pragma once

#include <string>

namespace FirmwareUpdate {

class GitHubUpdateClient {
public:
    static bool fetchText(const char* url, std::string& body, std::string& error);
};

} // namespace FirmwareUpdate

