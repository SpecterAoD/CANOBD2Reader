#pragma once

// Semantic version handling for firmware versions (SemVer 2.0 ordering).
//
// Accepted forms: "2.1.0", "V2.1.0", "2.1.0-beta.3", "V2.1.0-rc.1".
// Legacy tags of this project are normalised, so old devices keep working:
//   "2.0.0.b6" -> 2.0.0-beta.6      "2.0.0-Beta6" -> 2.0.0-beta.6
//   "1.0.15.test1" -> 1.0.15-test.1
// Ordering: 2.0.0-beta.9 < 2.0.0-beta.10 < 2.0.0-rc.1 < 2.0.0 < 2.0.1

#include <string>
#include <vector>

namespace FirmwareUpdate {

struct FirmwareVersion {
    bool valid = false;
    int major = 0;
    int minor = 0;
    int patch = 0;
    std::vector<std::string> prerelease; // empty for a final release

    static FirmwareVersion parse(const std::string& text);
    bool isPrerelease() const { return !prerelease.empty(); }
    // Canonical form without leading "V", e.g. "2.0.0-beta.6".
    std::string toString() const;
};

// <0 if left is older, 0 if equal, >0 if left is newer. Invalid versions sort
// before every valid version so that garbage never looks like an upgrade.
int compareVersions(const std::string& left, const std::string& right);

} // namespace FirmwareUpdate
