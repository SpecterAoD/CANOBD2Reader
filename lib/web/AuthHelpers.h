#pragma once

#include <cstddef>
#include "SecurityConfig.h"

#if defined(ARDUINO)
  #include <Arduino.h>
  #include <WebServer.h>
#endif

namespace WebSecurity {

bool constantTimeEquals(const char* left, const char* right);
bool authenticationEnabled();
const char* username();
const char* password();
const char* apiToken();
bool isConfiguredToken(const char* token);

#if defined(ARDUINO)
bool requireAuthentication(WebServer& server, bool protectEndpoint = true);
#endif

}
