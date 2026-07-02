#pragma once

#include <cstddef>
#include "config/SecurityConfig.h"

#if defined(ARDUINO)
  #include <Arduino.h>
  #include <WebServer.h>
#else
  #include <string>
  using String = std::string;
#endif

namespace WebSecurity {

bool constantTimeEquals(const char* left, const char* right);
bool authenticationEnabled();
const char* username();
const char* password();
const char* apiToken();
bool isConfiguredToken(const char* token);
String senderManagementSecurityWarning();
String displayManagementSecurityWarning();
String espNowSecurityWarning();
String targetSecurityWarning(const char* target, bool includeEspNow = true);
bool senderManagementConfigurationSafe();
bool displayManagementConfigurationSafe();
bool espNowConfigurationSafe();

#if defined(ARDUINO)
bool requireAuthentication(WebServer& server, bool protectEndpoint = true);
#endif

}
