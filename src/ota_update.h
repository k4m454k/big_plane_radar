#pragma once

#include <WebServer.h>

namespace OtaUpdate {

using PasswordFn = const char *(*)();
using StatusFn = void (*)(const char *title, const char *body);

/** Register /firmware and /firmware-upload on the existing web server. */
void attach(WebServer &server, PasswordFn password, StatusFn status);

bool inProgress();

}  // namespace OtaUpdate
