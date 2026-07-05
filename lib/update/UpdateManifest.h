#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "UpdateChannel.h"

namespace FirmwareUpdate {

enum class VersionRelation {
    Older,
    Same,
    Newer
};

enum class InstallEligibility {
    Installable,
    Installed,
    Rollback,
    WrongTarget,
    IncompatibleProtocol,
    MissingSha256,
    MissingUrl,
    ChannelBlocked
};

struct FirmwareVersionEntry {
    std::string version;
    UpdateChannel channel = UpdateChannel::Development;
    std::string url;
    std::string sha256;
    uint8_t protocol = 0;
    std::string target;
    std::string createdAt;
};

struct UpdateManifest {
    int manifestVersion = 0;
    std::string project;
    std::vector<FirmwareVersionEntry> sender;
    std::vector<FirmwareVersionEntry> display;
};

bool parseManifest(const std::string& json, UpdateManifest& manifest);
int compareVersions(const std::string& left, const std::string& right);
VersionRelation relationToInstalled(const std::string& installed, const std::string& candidate);
bool targetMatches(const FirmwareVersionEntry& entry, const char* expectedTarget);
InstallEligibility evaluateEntry(const FirmwareVersionEntry& entry,
                                 const char* expectedTarget,
                                 uint8_t protocolVersion,
                                 const char* installedVersion,
                                 UpdateChannel selectedChannel,
                                 bool allowRollback);
const FirmwareVersionEntry* findBestForwardUpdate(const UpdateManifest& manifest,
                                                  const char* target,
                                                  uint8_t protocolVersion,
                                                  const char* installedVersion,
                                                  UpdateChannel selectedChannel);
const FirmwareVersionEntry* findVersion(const UpdateManifest& manifest,
                                        const char* target,
                                        const char* version);
const char* eligibilityName(InstallEligibility value);
std::string manifestVersionsJson(const UpdateManifest& manifest,
                                 const char* target,
                                 uint8_t protocolVersion,
                                 const char* installedVersion,
                                 UpdateChannel selectedChannel,
                                 bool allowRollback);

} // namespace FirmwareUpdate

