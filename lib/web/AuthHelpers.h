#pragma once

// HTTP authentication for the management web servers (sender and display).
// Accepts HTTP Basic auth (browser) or the API token (scripts) via the
// "X-API-Token" header or "Authorization: Bearer <token>". Tokens in the URL
// are not accepted because they end up in browser history and logs.

#include <cstddef>

#if defined(ARDUINO)
  #include <Arduino.h>
  #include <WebServer.h>
#endif

namespace WebSecurity {

// Compares without early exit so response timing does not leak the secret.
bool constantTimeEquals(const char* left, const char* right);
bool isConfiguredToken(const char* token);

#if defined(ARDUINO)
// Headers the WebServer must collect for token authentication.
void collectAuthHeaders(WebServer& server);
// Returns true if the request may proceed; otherwise a 401 was already sent.
bool requireAuthentication(WebServer& server, bool protectEndpoint = true);
#endif

} // namespace WebSecurity
