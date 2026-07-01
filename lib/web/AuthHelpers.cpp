#include "AuthHelpers.h"

#include <cstring>

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

bool authenticationEnabled() {
    return SecurityConfig::EnableAuthentication;
}

const char* username() {
    return SecurityConfig::WebUsername;
}

const char* password() {
    return SecurityConfig::WebPassword;
}

const char* apiToken() {
    return SecurityConfig::ApiToken;
}

bool isConfiguredToken(const char* token) {
    if (token == nullptr || token[0] == '\0') return false;
    if (SecurityConfig::ApiToken == nullptr || SecurityConfig::ApiToken[0] == '\0') return false;
    return constantTimeEquals(token, SecurityConfig::ApiToken);
}

#if defined(ARDUINO)
namespace {
bool bearerTokenMatches(const String& authorizationHeader) {
    constexpr const char* prefix = "Bearer ";
    if (!authorizationHeader.startsWith(prefix)) return false;
    return WebSecurity::isConfiguredToken(authorizationHeader.substring(strlen(prefix)).c_str());
}
}

bool requireAuthentication(WebServer& server, bool protectEndpoint) {
    if (!protectEndpoint || !SecurityConfig::EnableAuthentication) return true;

    if (server.authenticate(SecurityConfig::WebUsername, SecurityConfig::WebPassword)) {
        return true;
    }

    if (isConfiguredToken(server.header("X-API-Token").c_str()) ||
        bearerTokenMatches(server.header("Authorization")) ||
        isConfiguredToken(server.arg("token").c_str())) {
        return true;
    }

    server.requestAuthentication(BASIC_AUTH, SecurityConfig::AuthenticationRealm);
    return false;
}
#endif

}
