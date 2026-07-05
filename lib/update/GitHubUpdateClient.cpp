#include "GitHubUpdateClient.h"

#include "config/UpdateConfig.h"

#if defined(ARDUINO)
  #include <HTTPClient.h>
  #include <WiFiClientSecure.h>
#endif

namespace FirmwareUpdate {

bool GitHubUpdateClient::fetchText(const char* url, std::string& body, std::string& error) {
    body.clear();
    error.clear();
#if defined(ARDUINO)
    WiFiClientSecure client;
    if (UpdateConfig::RequireTlsCertificateValidation) {
        client.setCACert(UpdateConfig::GitHubRootCa);
    } else if (UpdateConfig::AllowInsecureTlsForDevelopment) {
        client.setInsecure();
    } else {
        error = "TLS validation disabled by config";
        return false;
    }

    HTTPClient http;
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    http.setTimeout(UpdateConfig::ManifestTimeoutMs);
    if (!http.begin(client, url)) {
        error = "HTTP begin failed";
        return false;
    }
    const int code = http.GET();
    if (code != HTTP_CODE_OK) {
        error = "HTTP status " + std::to_string(code);
        http.end();
        return false;
    }
    String payload = http.getString();
    body.assign(payload.c_str(), payload.length());
    http.end();
    return true;
#else
    (void)url;
    error = "GitHub fetch is only available on Arduino firmware";
    return false;
#endif
}

} // namespace FirmwareUpdate

