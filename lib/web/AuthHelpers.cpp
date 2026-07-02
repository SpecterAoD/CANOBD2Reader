#include "AuthHelpers.h"

#include <cstring>

#include "config/NetworkConfig.h"
#include "secrets_defaults.h"

namespace WebSecurity {

namespace {

bool stringEquals(const char* left, const char* right) {
    return WebSecurity::constantTimeEquals(left, right);
}

bool hasExampleEspNowKey() {
    return std::memcmp(NetworkConfig::EspNowAesKey,
                       SecretsExampleDefaults::EspNowAesKey,
                       sizeof(SecretsExampleDefaults::EspNowAesKey)) == 0;
}

void appendWarning(String& warning, const char* issue) {
    if (warning.length() > 0) warning += "; ";
    warning += issue;
}

bool targetIs(const char* target, const char* expected) {
    return stringEquals(target, expected);
}

} // namespace

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

String senderManagementSecurityWarning() {
    String warning;
    if (!SecurityConfig::BlockNetworkFeaturesOnPlaceholderSecrets) return warning;

    if (stringEquals(NetworkConfig::SenderWebPassword, SecretsExampleDefaults::SenderWebPassword)) {
        appendWarning(warning, "Sender-AP-Passwort ist noch Platzhalter");
    }
    if (stringEquals(NetworkConfig::SenderOtaPassword, SecretsExampleDefaults::SenderOtaPassword)) {
        appendWarning(warning, "Sender-OTA-Passwort ist noch Platzhalter");
    }
    if (stringEquals(SecurityConfig::WebPassword, SecretsExampleDefaults::WebPassword)) {
        appendWarning(warning, "Web-Passwort ist noch Platzhalter");
    }
    if (stringEquals(SecurityConfig::ApiToken, SecretsExampleDefaults::ApiToken)) {
        appendWarning(warning, "API-Token ist noch Platzhalter");
    }
    return warning;
}

String displayManagementSecurityWarning() {
    String warning;
    if (!SecurityConfig::BlockNetworkFeaturesOnPlaceholderSecrets) return warning;

    if (stringEquals(NetworkConfig::DisplayWebPassword, SecretsExampleDefaults::DisplayWebPassword)) {
        appendWarning(warning, "Display-AP-Passwort ist noch Platzhalter");
    }
    if (stringEquals(SecurityConfig::WebPassword, SecretsExampleDefaults::WebPassword)) {
        appendWarning(warning, "Web-Passwort ist noch Platzhalter");
    }
    if (stringEquals(SecurityConfig::ApiToken, SecretsExampleDefaults::ApiToken)) {
        appendWarning(warning, "API-Token ist noch Platzhalter");
    }
    return warning;
}

String espNowSecurityWarning() {
    String warning;
    if (!SecurityConfig::BlockNetworkFeaturesOnPlaceholderSecrets) return warning;

    if (hasExampleEspNowKey()) {
        appendWarning(warning, "ESP-NOW-Key ist noch Platzhalter");
    }
    return warning;
}

String targetSecurityWarning(const char* target, bool includeEspNow) {
    String warning;

    if (targetIs(target, "sender")) {
        warning = senderManagementSecurityWarning();
    } else if (targetIs(target, "display")) {
        warning = displayManagementSecurityWarning();
    }

    if (includeEspNow) {
        const String espNowWarning = espNowSecurityWarning();
        if (espNowWarning.length() > 0) {
            if (warning.length() > 0) warning += "; ";
            warning += espNowWarning;
        }
    }
    return warning;
}

bool senderManagementConfigurationSafe() {
    return senderManagementSecurityWarning().length() == 0;
}

bool displayManagementConfigurationSafe() {
    return displayManagementSecurityWarning().length() == 0;
}

bool espNowConfigurationSafe() {
    return espNowSecurityWarning().length() == 0;
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
