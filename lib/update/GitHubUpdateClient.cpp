#include "GitHubUpdateClient.h"

#include "OtaSession.h"
#include "SystemHealth.h"
#include "config/UpdateConfig.h"

#if defined(ARDUINO)
  #include <HTTPClient.h>
  #include <NetworkClientSecure.h>
#endif

namespace FirmwareUpdate {

#if defined(ARDUINO)
namespace {

// Opens a GET request; on success the caller owns `http` and must call end().
bool openGet(NetworkClientSecure& client, HTTPClient& http, const char* url,
             uint32_t timeoutMs, std::string& error) {
    client.setCACert(UpdateConfig::GitHubRootCaBundle);
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    http.setTimeout(timeoutMs);
    if (!http.begin(client, url)) {
        error = "HTTP-Verbindung fehlgeschlagen";
        return false;
    }
    const int code = http.GET();
    if (code != HTTP_CODE_OK) {
        error = code < 0 ? std::string("HTTP: ") + HTTPClient::errorToString(code).c_str()
                         : "HTTP-Status " + std::to_string(code);
        http.end();
        return false;
    }
    return true;
}

} // namespace
#endif

bool GitHubUpdateClient::fetchText(const char* url, std::size_t maxBytes, std::string& body, std::string& error) {
    body.clear();
    error.clear();
#if defined(ARDUINO)
    NetworkClientSecure client;
    HTTPClient http;
    if (!openGet(client, http, url, UpdateConfig::ManifestTimeoutMs, error)) return false;

    const int size = http.getSize();
    if (size > 0 && static_cast<std::size_t>(size) > maxBytes) {
        error = "Antwort zu gross";
        http.end();
        return false;
    }
    String payload = http.getString();
    http.end();
    if (payload.length() > maxBytes) {
        error = "Antwort zu gross";
        return false;
    }
    body.assign(payload.c_str(), payload.length());
    return true;
#else
    (void)url;
    (void)maxBytes;
    error = "GitHub-Zugriff nur auf dem Geraet verfuegbar";
    return false;
#endif
}

bool GitHubUpdateClient::downloadInto(const char* url, OtaSession& session, std::string& error) {
    error.clear();
#if defined(ARDUINO)
    NetworkClientSecure client;
    HTTPClient http;
    if (!openGet(client, http, url, UpdateConfig::DownloadTimeoutMs, error)) return false;

    const int contentLength = http.getSize();
    if (contentLength > 0 && static_cast<std::size_t>(contentLength) > ESP.getFreeSketchSpace()) {
        error = "Firmware groesser als OTA-Partition";
        http.end();
        return false;
    }

    NetworkClient* stream = http.getStreamPtr();
    uint8_t buffer[1024];
    std::size_t received = 0;
    uint32_t lastDataMs = millis();
    while (http.connected() && (contentLength < 0 || received < static_cast<std::size_t>(contentLength))) {
        SystemHealth::feed();  // a download takes longer than the loop watchdog allows
        const std::size_t available = stream->available();
        if (available == 0) {
            if (millis() - lastDataMs > UpdateConfig::DownloadTimeoutMs) {
                error = "Download-Timeout";
                http.end();
                return false;
            }
            delay(5);
            continue;
        }
        const int read = stream->readBytes(buffer, available > sizeof(buffer) ? sizeof(buffer) : available);
        if (read <= 0) continue;
        lastDataMs = millis();
        if (!session.write(buffer, static_cast<std::size_t>(read))) {
            error = session.error();
            http.end();
            return false;
        }
        received += static_cast<std::size_t>(read);
    }
    http.end();

    if (contentLength > 0 && received != static_cast<std::size_t>(contentLength)) {
        error = "Download unvollstaendig";
        return false;
    }
    return true;
#else
    (void)url;
    (void)session;
    error = "GitHub-Zugriff nur auf dem Geraet verfuegbar";
    return false;
#endif
}

} // namespace FirmwareUpdate
