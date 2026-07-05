#include "UpdateManifest.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>

namespace FirmwareUpdate {

namespace {

std::string trimVersion(std::string value) {
    if (!value.empty() && (value[0] == 'V' || value[0] == 'v')) value.erase(value.begin());
    return value;
}

bool equalsIgnoreCase(const char* a, const char* b) {
    if (a == nullptr || b == nullptr) return false;
    while (*a != '\0' && *b != '\0') {
        char ca = *a++;
        char cb = *b++;
        if (ca >= 'A' && ca <= 'Z') ca = static_cast<char>(ca - 'A' + 'a');
        if (cb >= 'A' && cb <= 'Z') cb = static_cast<char>(cb - 'A' + 'a');
        if (ca != cb) return false;
    }
    return *a == '\0' && *b == '\0';
}

std::string jsonEscape(const std::string& value) {
    std::string out;
    out.reserve(value.size() + 8);
    for (char c : value) {
        if (c == '"' || c == '\\') {
            out.push_back('\\');
            out.push_back(c);
        } else if (c == '\n') {
            out += "\\n";
        } else if (c == '\r') {
            out += "\\r";
        } else {
            out.push_back(c);
        }
    }
    return out;
}

std::string extractString(const std::string& object, const char* key) {
    const std::string pattern = std::string("\"") + key + "\"";
    size_t pos = object.find(pattern);
    if (pos == std::string::npos) return {};
    pos = object.find(':', pos);
    if (pos == std::string::npos) return {};
    pos = object.find('"', pos);
    if (pos == std::string::npos) return {};
    ++pos;
    std::string out;
    bool escape = false;
    for (; pos < object.size(); ++pos) {
        const char c = object[pos];
        if (escape) {
            out.push_back(c);
            escape = false;
        } else if (c == '\\') {
            escape = true;
        } else if (c == '"') {
            break;
        } else {
            out.push_back(c);
        }
    }
    return out;
}

int extractInt(const std::string& object, const char* key) {
    const std::string pattern = std::string("\"") + key + "\"";
    size_t pos = object.find(pattern);
    if (pos == std::string::npos) return 0;
    pos = object.find(':', pos);
    if (pos == std::string::npos) return 0;
    ++pos;
    while (pos < object.size() && std::isspace(static_cast<unsigned char>(object[pos]))) ++pos;
    return std::atoi(object.c_str() + pos);
}

bool extractArrayObjects(const std::string& json, const char* key, std::vector<std::string>& objects) {
    const std::string pattern = std::string("\"") + key + "\"";
    size_t pos = json.find(pattern);
    if (pos == std::string::npos) return false;
    pos = json.find('[', pos);
    if (pos == std::string::npos) return false;
    int depth = 0;
    size_t objectStart = std::string::npos;
    bool inString = false;
    bool escape = false;
    for (; pos < json.size(); ++pos) {
        const char c = json[pos];
        if (inString) {
            if (escape) escape = false;
            else if (c == '\\') escape = true;
            else if (c == '"') inString = false;
            continue;
        }
        if (c == '"') {
            inString = true;
        } else if (c == '{') {
            if (depth == 0) objectStart = pos;
            ++depth;
        } else if (c == '}') {
            --depth;
            if (depth == 0 && objectStart != std::string::npos) {
                objects.push_back(json.substr(objectStart, pos - objectStart + 1));
                objectStart = std::string::npos;
            }
        } else if (c == ']' && depth == 0) {
            return true;
        }
    }
    return false;
}

void parseEntries(const std::string& json, const char* key, const char* target, std::vector<FirmwareVersionEntry>& out) {
    std::vector<std::string> objects;
    if (!extractArrayObjects(json, key, objects)) return;
    for (const auto& object : objects) {
        FirmwareVersionEntry entry;
        entry.version = extractString(object, "version");
        entry.channel = parseChannel(extractString(object, "channel").c_str());
        entry.url = extractString(object, "url");
        entry.sha256 = extractString(object, "sha256");
        entry.protocol = static_cast<uint8_t>(extractInt(object, "protocol"));
        entry.target = extractString(object, "target");
        if (entry.target.empty()) entry.target = target;
        entry.createdAt = extractString(object, "createdAt");
        if (!entry.version.empty()) out.push_back(entry);
    }
}

std::vector<int> numericParts(const std::string& value) {
    std::string v = trimVersion(value);
    std::vector<int> parts;
    size_t pos = 0;
    while (pos < v.size() && parts.size() < 4) {
        if (!std::isdigit(static_cast<unsigned char>(v[pos]))) break;
        char* end = nullptr;
        long n = std::strtol(v.c_str() + pos, &end, 10);
        parts.push_back(static_cast<int>(n));
        pos = static_cast<size_t>(end - v.c_str());
        if (pos < v.size() && v[pos] == '.') ++pos;
        else break;
    }
    while (parts.size() < 4) parts.push_back(0);
    return parts;
}

} // namespace

bool parseManifest(const std::string& json, UpdateManifest& manifest) {
    manifest = {};
    manifest.manifestVersion = extractInt(json, "manifestVersion");
    manifest.project = extractString(json, "project");
    parseEntries(json, "sender", "sender", manifest.sender);
    parseEntries(json, "display", "display", manifest.display);
    return manifest.manifestVersion > 0 && (!manifest.sender.empty() || !manifest.display.empty());
}

int compareVersions(const std::string& left, const std::string& right) {
    const auto a = numericParts(left);
    const auto b = numericParts(right);
    for (size_t i = 0; i < a.size(); ++i) {
        if (a[i] < b[i]) return -1;
        if (a[i] > b[i]) return 1;
    }
    const std::string la = trimVersion(left);
    const std::string rb = trimVersion(right);
    if (la == rb) return 0;
    return la < rb ? -1 : 1;
}

VersionRelation relationToInstalled(const std::string& installed, const std::string& candidate) {
    const int cmp = compareVersions(candidate, installed);
    if (cmp < 0) return VersionRelation::Older;
    if (cmp > 0) return VersionRelation::Newer;
    return VersionRelation::Same;
}

bool targetMatches(const FirmwareVersionEntry& entry, const char* expectedTarget) {
    return equalsIgnoreCase(entry.target.c_str(), expectedTarget);
}

InstallEligibility evaluateEntry(const FirmwareVersionEntry& entry,
                                 const char* expectedTarget,
                                 uint8_t protocolVersion,
                                 const char* installedVersion,
                                 UpdateChannel selectedChannel,
                                 bool allowRollback) {
    if (!targetMatches(entry, expectedTarget)) return InstallEligibility::WrongTarget;
    if (!channelAllows(selectedChannel, entry.channel)) return InstallEligibility::ChannelBlocked;
    if (entry.protocol != protocolVersion) return InstallEligibility::IncompatibleProtocol;
    if (entry.sha256.empty()) return InstallEligibility::MissingSha256;
    if (entry.url.empty()) return InstallEligibility::MissingUrl;

    const auto relation = relationToInstalled(installedVersion == nullptr ? "" : installedVersion, entry.version);
    if (relation == VersionRelation::Same) return InstallEligibility::Installed;
    if (relation == VersionRelation::Older) {
        return allowRollback ? InstallEligibility::Rollback : InstallEligibility::Installed;
    }
    return InstallEligibility::Installable;
}

const FirmwareVersionEntry* findBestForwardUpdate(const UpdateManifest& manifest,
                                                  const char* target,
                                                  uint8_t protocolVersion,
                                                  const char* installedVersion,
                                                  UpdateChannel selectedChannel) {
    const auto& list = equalsIgnoreCase(target, "display") ? manifest.display : manifest.sender;
    const FirmwareVersionEntry* best = nullptr;
    for (const auto& entry : list) {
        if (evaluateEntry(entry, target, protocolVersion, installedVersion, selectedChannel, false) !=
            InstallEligibility::Installable) {
            continue;
        }
        if (best == nullptr || compareVersions(entry.version, best->version) > 0) best = &entry;
    }
    return best;
}

const FirmwareVersionEntry* findVersion(const UpdateManifest& manifest,
                                        const char* target,
                                        const char* version) {
    const auto& list = equalsIgnoreCase(target, "display") ? manifest.display : manifest.sender;
    for (const auto& entry : list) {
        if (entry.version == version) return &entry;
    }
    return nullptr;
}

const char* eligibilityName(InstallEligibility value) {
    switch (value) {
        case InstallEligibility::Installable: return "newer";
        case InstallEligibility::Installed: return "installed";
        case InstallEligibility::Rollback: return "rollback";
        case InstallEligibility::WrongTarget: return "wrong_target";
        case InstallEligibility::IncompatibleProtocol: return "incompatible_protocol";
        case InstallEligibility::MissingSha256: return "missing_sha256";
        case InstallEligibility::MissingUrl: return "missing_url";
        case InstallEligibility::ChannelBlocked: return "channel_blocked";
    }
    return "unknown";
}

std::string manifestVersionsJson(const UpdateManifest& manifest,
                                 const char* target,
                                 uint8_t protocolVersion,
                                 const char* installedVersion,
                                 UpdateChannel selectedChannel,
                                 bool allowRollback) {
    const auto& list = equalsIgnoreCase(target, "display") ? manifest.display : manifest.sender;
    std::string json = "{\"versions\":[";
    bool first = true;
    for (const auto& entry : list) {
        if (!first) json += ",";
        first = false;
        const auto eligibility = evaluateEntry(entry, target, protocolVersion, installedVersion, selectedChannel, allowRollback);
        json += "{";
        json += "\"version\":\"" + jsonEscape(entry.version) + "\",";
        json += "\"channel\":\"" + std::string(channelName(entry.channel)) + "\",";
        json += "\"createdAt\":\"" + jsonEscape(entry.createdAt) + "\",";
        json += "\"target\":\"" + jsonEscape(entry.target) + "\",";
        char number[16];
        std::snprintf(number, sizeof(number), "%u", entry.protocol);
        json += "\"protocol\":" + std::string(number) + ",";
        json += "\"status\":\"" + std::string(eligibilityName(eligibility)) + "\",";
        json += "\"sha256\":\"" + jsonEscape(entry.sha256) + "\"";
        json += "}";
    }
    json += "]}";
    return json;
}

} // namespace FirmwareUpdate

