#include "ota_update.h"

#include <Update.h>

#include "app_log.h"
#include "app_watchdog.h"

namespace OtaUpdate {
namespace {

constexpr char kUsername[] = "admin";

constexpr char kFirmwarePage[] PROGMEM = R"HTML(
<!doctype html><html lang="en"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Plane Radar firmware</title>
<style>
body{font-family:system-ui,sans-serif;background:#050805;color:#e8ffe8;margin:0}
main{max-width:34rem;margin:3rem auto;padding:1.5rem}
.card{background:#101512;border:1px solid #295;padding:1.5rem}
input,button{box-sizing:border-box;width:100%;margin-top:1rem;padding:.8rem}
button{background:#19d45a;color:#001b08;border:0;font-weight:700}
small{color:#8a9}a{color:#73ff8a}
</style></head><body><main><div class="card">
<h1>Firmware update</h1>
<p>Upload the application <strong>.bin</strong> from a source build
(<code>big_plane_radar.ino.bin</code>), not the merged flash image.</p>
<form method="post" action="/firmware-upload" enctype="multipart/form-data">
<input type="file" name="firmware" accept=".bin,application/octet-stream" required>
<button type="submit">Install and restart</button></form>
<p><small>Do not upload <code>.merged.bin</code> here. Keep power connected
until the display restarts.</small></p>
<p><a href="/">Back to settings</a></p>
</div></main></body></html>
)HTML";

WebServer *s_server = nullptr;
PasswordFn s_password = nullptr;
StatusFn s_status = nullptr;
bool s_in_progress = false;
bool s_upload_authenticated = false;
String s_upload_error;

const char *password() {
    if (s_password == nullptr) {
        return "plane-radar";
    }
    const char *value = s_password();
    return (value != nullptr && value[0] != '\0') ? value : "plane-radar";
}

bool requestAuthenticated() {
    if (s_server == nullptr) {
        return false;
    }
    if (s_server->authenticate(kUsername, password())) {
        return true;
    }
    s_server->requestAuthentication();
    return false;
}

void showFirmwarePage() {
    if (s_server == nullptr || !requestAuthenticated()) {
        return;
    }
    s_server->send_P(200, PSTR("text/html"), kFirmwarePage);
}

void recordUpdateError() {
    s_upload_error = Update.errorString();
    if (s_upload_error.length() == 0) {
        s_upload_error = "Unknown flash error";
    }
    Update.end();
    RADAR_LOGE("OTA failed: %s\n", s_upload_error.c_str());
}

void handleUploadChunk() {
    if (s_server == nullptr) {
        return;
    }
    HTTPUpload &upload = s_server->upload();
    AppWatchdog::feed();

    if (upload.status == UPLOAD_FILE_START) {
        s_upload_authenticated = s_server->authenticate(kUsername, password());
        s_upload_error.clear();
        if (!s_upload_authenticated) {
            return;
        }
        s_in_progress = true;
        if (!upload.filename.endsWith(".bin") ||
            upload.filename.indexOf("merged") >= 0) {
            s_upload_error = "Upload the application .bin, not the merged image";
            return;
        }
        if (s_status != nullptr) {
            s_status("FIRMWARE UPDATE", "Keep power connected.");
        }
        RADAR_LOGI("OTA: receiving %s\n", upload.filename.c_str());
        if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) {
            recordUpdateError();
        }
        return;
    }

    if (!s_upload_authenticated || s_upload_error.length() != 0) {
        return;
    }

    if (upload.status == UPLOAD_FILE_WRITE) {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
            recordUpdateError();
        }
    } else if (upload.status == UPLOAD_FILE_END) {
        if (!Update.end(true)) {
            recordUpdateError();
        } else {
            RADAR_LOGI("OTA: wrote %u bytes\n", upload.totalSize);
        }
    } else if (upload.status == UPLOAD_FILE_ABORTED) {
        s_upload_error = "Upload aborted";
        Update.end();
    }
}

void handleUploadDone() {
    if (s_server == nullptr) {
        return;
    }
    if (!s_upload_authenticated) {
        s_in_progress = false;
        s_server->requestAuthentication();
        return;
    }

    if (s_upload_error.length() != 0 || Update.hasError()) {
        s_in_progress = false;
        const String message =
            String("<!doctype html><meta name=viewport content='width=device-width'>"
                   "<h1>Update failed</h1><p>") +
            (s_upload_error.length() != 0 ? s_upload_error : Update.errorString()) +
            "</p><p><a href='/firmware'>Try again</a></p>";
        s_server->send(500, "text/html", message);
        return;
    }

    s_server->client().setNoDelay(true);
    s_server->send(
        200, "text/html",
        "<!doctype html><meta name=viewport content='width=device-width'>"
        "<h1>Update installed</h1><p>Plane Radar is restarting...</p>");
    delay(250);
    s_server->client().stop();
    ESP.restart();
}

}  // namespace

void attach(WebServer &server, PasswordFn passwordFn, StatusFn status) {
    s_server = &server;
    s_password = passwordFn;
    s_status = status;
    server.on("/firmware", HTTP_GET, showFirmwarePage);
    server.on("/firmware-upload", HTTP_POST, handleUploadDone, handleUploadChunk);
}

bool inProgress() { return s_in_progress; }

}  // namespace OtaUpdate
