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

#if defined(ARDUINO)
bool requireAuthentication(WebServer& server, bool protectEndpoint) {
    if (!protectEndpoint || !SecurityConfig::EnableAuthentication) return true;

    if (server.authenticate(SecurityConfig::WebUsername, SecurityConfig::WebPassword)) {
        return true;
    }

    server.requestAuthentication(BASIC_AUTH, SecurityConfig::AuthenticationRealm);
    return false;
}
#endif

}
