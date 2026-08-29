// WebUI.ino
// Per-board HTTP control surface: mDNS hostname + live status.
// LEFT board additionally serves the control page and /toggle (see below).

#include <WebServer.h>
#include <ESPmDNS.h>

WebServer webServer(80);

static String jsonState() {
  Sample s = readSample();
  bool peerFresh = (peerArm != '?') && (millis() - peerRxMs < FB_PEER_TIMEOUT);
  String json = "{";
  json += "\"arm\":\""        + String(ARM_CHAR) + "\",";
  json += "\"motorEnabled\":" + String(g_motorEnabled ? "true" : "false") + ",";
  json += "\"peerFresh\":"    + String(peerFresh ? "true" : "false") + ",";
  json += "\"tempC\":"        + String(s.tempC, 1) + ",";
  json += "\"uptimeMs\":"     + String(millis());
  json += "}";
  return json;
}

void handleState() {
  webServer.sendHeader("Access-Control-Allow-Origin", "*");
  webServer.send(200, "application/json", jsonState());
}

void webUiBegin() {
  String host = String("bb-") + (I_AM_LEFT ? "left" : "right");
  if (!MDNS.begin(host.c_str())) {
    Serial.println(F("mDNS init failed (use the IP printed above instead)"));
  } else {
    Serial.printf("mDNS ready: http://%s.local/\n", host.c_str());
  }

  webServer.on("/state", HTTP_GET, handleState);
  webServer.begin();
}

void webUiHandleClient() {
  webServer.handleClient();
}
