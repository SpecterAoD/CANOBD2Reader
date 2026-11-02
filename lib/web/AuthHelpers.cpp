#include "AuthHelpers.h"

#include <cstring>

#include "DeviceSecrets.h"
#include "config/SecurityConfig.h"

namespace WebSecurity {

bool constantTimeEquals(const char* left, const char* right) {
    if (left == nullptr || right == nullptr) return false;

    const std::size_t leftLength = std::strlen(left);
    const std::size_t rightLength = std::strlen(right);
    const std::size_t maxLength = leftLength > rightLength ? leftLength : rightLength;

    unsigned char diff = static_cast<unsigned char>(leftLength ^ rightLength);
    for (std::size_t index = 0; index < maxLength; ++index) {
        const unsigned char a = index < leftLength ? static_cast<unsigned char>(left[index]) : 0;
        const unsigned char b = index < rightLength ? static_cast<unsigned char>(right[index]) : 0;
        diff |= static_cast<unsigned char>(a ^ b);
    }
    return diff == 0;
}

bool isConfiguredToken(const char* token) {
    if (!Security::provisioned()) return false;
    if (token == nullptr || token[0] == '\0') return false;
    return constantTimeEquals(token, Security::secrets().apiToken);
}

#if defined(ARDUINO)
void collectAuthHeaders(WebServer& server) {
    static const char* headers[] = {"X-API-Token", "Authorization", "X-Maintenance-Confirm"};
    server.collectHeaders(headers, 3);
}

bool requireAuthentication(WebServer& server, bool protectEndpoint) {
    if (!protectEndpoint || !SecurityConfig::EnableAuthentication) return true;

    // Without provisioned credentials the servers are not started at all; this
    // is only a second line of defence.
    if (Security::provisioned()) {
        const Security::DeviceSecrets& secrets = Security::secrets();
        if (server.authenticate(secrets.webUser, secrets.webPassword)) return true;

        if (isConfiguredToken(server.header("X-API-Token").c_str())) return true;
        const String authorization = server.header("Authorization");
        if (authorization.startsWith("Bearer ") && isConfiguredToken(authorization.c_str() + 7)) return true;
    }

    server.requestAuthentication(BASIC_AUTH, SecurityConfig::AuthenticationRealm);
    return false;
}
#endif

} // namespace WebSecurity
