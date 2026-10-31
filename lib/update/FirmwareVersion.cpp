#include "FirmwareVersion.h"

#include <cctype>
#include <cstdlib>

namespace FirmwareUpdate {
namespace {

bool isDigits(const std::string& text) {
    if (text.empty()) return false;
    for (char c : text) {
        if (!std::isdigit(static_cast<unsigned char>(c))) return false;
    }
    return true;
}

std::string lower(std::string text) {
    for (char& c : text) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return text;
}

// "beta6" -> {"beta", "6"}; "b6" -> {"beta", "6"}; "rc" -> {"rc"}
void appendIdentifier(const std::string& raw, std::vector<std::string>& out) {
    std::size_t index = 0;
    while (index < raw.size()) {
        const bool digit = std::isdigit(static_cast<unsigned char>(raw[index])) != 0;
        std::size_t end = index;
        while (end < raw.size() && (std::isdigit(static_cast<unsigned char>(raw[end])) != 0) == digit) ++end;
        std::string part = raw.substr(index, end - index);
        if (part == "b") part = "beta";
        if (part == "a") part = "alpha";
        if (digit) part = std::to_string(std::atoi(part.c_str())); // strip leading zeros
        out.push_back(part);
        index = end;
    }
}

bool parseNumber(const std::string& text, std::size_t& pos, int& value) {
    const std::size_t start = pos;
    while (pos < text.size() && std::isdigit(static_cast<unsigned char>(text[pos]))) ++pos;
    if (pos == start || pos - start > 6) return false;
    value = std::atoi(text.substr(start, pos - start).c_str());
    return true;
}

int compareIdentifier(const std::string& a, const std::string& b) {
    const bool aNum = isDigits(a);
    const bool bNum = isDigits(b);
    if (aNum && bNum) {
        const long x = std::atol(a.c_str());
        const long y = std::atol(b.c_str());
        return x < y ? -1 : (x > y ? 1 : 0);
    }
    if (aNum != bNum) return aNum ? -1 : 1; // numeric identifiers sort first (SemVer 11.4.3)
    return a < b ? -1 : (a > b ? 1 : 0);
}

} // namespace

FirmwareVersion FirmwareVersion::parse(const std::string& input) {
    FirmwareVersion version;
    std::string text = input;
    while (!text.empty() && std::isspace(static_cast<unsigned char>(text.back()))) text.pop_back();
    std::size_t pos = 0;
    while (pos < text.size() && std::isspace(static_cast<unsigned char>(text[pos]))) ++pos;
    if (pos < text.size() && (text[pos] == 'V' || text[pos] == 'v')) ++pos;

    if (!parseNumber(text, pos, version.major) || pos >= text.size() || text[pos] != '.') return {};
    ++pos;
    if (!parseNumber(text, pos, version.minor) || pos >= text.size() || text[pos] != '.') return {};
    ++pos;
    if (!parseNumber(text, pos, version.patch)) return {};

    if (pos < text.size()) {
        // "-beta.1" (SemVer) or ".b6" (legacy) start a prerelease; "+build" is ignored.
        if (text[pos] == '+') {
            version.valid = true;
            return version;
        }
        if (text[pos] != '-' && text[pos] != '.') return {};
        ++pos;
        const std::size_t buildPos = text.find('+', pos);
        const std::string pre = lower(text.substr(pos, buildPos == std::string::npos ? std::string::npos : buildPos - pos));
        std::size_t start = 0;
        while (start <= pre.size()) {
            std::size_t end = pre.find_first_of(".-", start);
            if (end == std::string::npos) end = pre.size();
            const std::string id = pre.substr(start, end - start);
            if (id.empty()) return {};
            for (char c : id) {
                if (!std::isalnum(static_cast<unsigned char>(c))) return {};
            }
            appendIdentifier(id, version.prerelease);
            start = end + 1;
        }
    }
    version.valid = true;
    return version;
}

std::string FirmwareVersion::toString() const {
    if (!valid) return {};
    std::string out = std::to_string(major) + "." + std::to_string(minor) + "." + std::to_string(patch);
    for (std::size_t index = 0; index < prerelease.size(); ++index) {
        out += index == 0 ? "-" : ".";
        out += prerelease[index];
    }
    return out;
}

int compareVersions(const std::string& left, const std::string& right) {
    const FirmwareVersion a = FirmwareVersion::parse(left);
    const FirmwareVersion b = FirmwareVersion::parse(right);
    if (!a.valid || !b.valid) {
        if (a.valid == b.valid) return 0;
        return a.valid ? 1 : -1;
    }
    if (a.major != b.major) return a.major < b.major ? -1 : 1;
    if (a.minor != b.minor) return a.minor < b.minor ? -1 : 1;
    if (a.patch != b.patch) return a.patch < b.patch ? -1 : 1;

    // A final release is newer than any prerelease of the same version.
    if (a.prerelease.empty() || b.prerelease.empty()) {
        if (a.prerelease.empty() && b.prerelease.empty()) return 0;
        return a.prerelease.empty() ? 1 : -1;
    }
    const std::size_t count = a.prerelease.size() < b.prerelease.size() ? a.prerelease.size() : b.prerelease.size();
    for (std::size_t index = 0; index < count; ++index) {
        const int cmp = compareIdentifier(a.prerelease[index], b.prerelease[index]);
        if (cmp != 0) return cmp;
    }
    if (a.prerelease.size() == b.prerelease.size()) return 0;
    return a.prerelease.size() < b.prerelease.size() ? -1 : 1;
}

} // namespace FirmwareUpdate
