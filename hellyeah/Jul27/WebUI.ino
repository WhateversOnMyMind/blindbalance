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
  json += "\"rawLogging\":"   + String(rawLogActive() ? "true" : "false") + ",";
  json += "\"rawLogBytes\":"  + String((unsigned long)rawLogBytes()) + ",";
  json += "\"tempC\":"        + String(s.tempC, 1) + ",";
  json += "\"uptimeMs\":"     + String(millis());
  json += "}";
  return json;
}

static void postToggleEvent(bool state) {
  if (WiFi.status() != WL_CONNECTED) return;
  HTTPClient http;
  http.begin(scriptURL);
  http.addHeader("Content-Type", "application/json");
  String json = "{\"event\":\"motor_toggle\",\"state\":\"" +
                String(state ? "on" : "off") + "\",\"ts\":" + String((uint32_t)time(nullptr)) + "}";
  http.POST(json);
  http.end();
}

void handleState() {
  webServer.sendHeader("Access-Control-Allow-Origin", "*");
  webServer.send(200, "application/json", jsonState());
}

void handleToggle() {
  g_motorEnabled = !g_motorEnabled;
  postToggleEvent(g_motorEnabled);
  webServer.sendHeader("Access-Control-Allow-Origin", "*");
  webServer.send(200, "application/json", jsonState());
}

static const char CONTROL_PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head><meta name="viewport" content="width=device-width, initial-scale=1">
<title>BlindBalance Control</title>
<style>
body{font-family:sans-serif;text-align:center;padding:1em;background:#111;color:#eee}
button{font-size:1.5em;padding:0.6em 1.2em;border-radius:0.5em;border:none}
.on{background:#2ecc71}.off{background:#e74c3c}
.tile{margin:1em auto;padding:0.8em;border-radius:0.5em;background:#222;max-width:20em}
.stale{opacity:0.4}
.rawctl{margin-top:0.5em;font-size:0.8em}
.rawctl button{font-size:0.9em;padding:0.3em 0.6em;margin:0.2em;background:#333;color:#eee}
.rawctl a{color:#3498db;margin-left:0.5em}
</style></head><body>
<h2>Motor Feedback</h2>
<button id="toggleBtn" onclick="toggle()">...</button>
<div class="tile" id="tileL"><b>Left</b><div id="infoL">loading...</div>
  <div class="rawctl">
    <button onclick="rawlog('','start')">Start log</button>
    <button onclick="rawlog('','stop')">Stop log</button>
    <a href="/rawlog/download">Download</a>
    <button onclick="rawlog('','clear')">Clear</button>
  </div>
</div>
<div class="tile" id="tileR"><b>Right</b><div id="infoR">loading...</div>
  <div class="rawctl">
    <button onclick="rawlog('http://bb-right.local','start')">Start log</button>
    <button onclick="rawlog('http://bb-right.local','stop')">Stop log</button>
    <a href="http://bb-right.local/rawlog/download">Download</a>
    <button onclick="rawlog('http://bb-right.local','clear')">Clear</button>
  </div>
</div>
<script>
let isPolling = false;
async function poll(){
  if (isPolling) return;
  isPolling = true;
  try{
    const l = await (await fetch('/state')).json();
    render(l, 'infoL', 'tileL');
    setBtn(l.motorEnabled);
  }catch(e){ document.getElementById('tileL').classList.add('stale'); }
  try{
    const r = await (await fetch('http://bb-right.local/state')).json();
    render(r, 'infoR', 'tileR');
  }catch(e){ document.getElementById('tileR').classList.add('stale'); }
  isPolling = false;
}
function render(s, infoId, tileId){
  document.getElementById(infoId).innerText =
    'motor: ' + (s.motorEnabled ? 'on' : 'off') +
    ' | peer: ' + (s.peerFresh ? 'ok' : 'stale') +
    ' | temp: ' + s.tempC + 'C' +
    ' | rawlog: ' + (s.rawLogging ? ('recording ' + s.rawLogBytes + 'B') : 'idle');
  document.getElementById(tileId).classList.remove('stale');
}
function setBtn(on){
  const b = document.getElementById('toggleBtn');
  b.innerText = on ? 'Motor: ON (tap to disable)' : 'Motor: OFF (tap to enable)';
  b.className = on ? 'on' : 'off';
}
async function toggle(){
  const s = await (await fetch('/toggle', {method:'POST'})).json();
  setBtn(s.motorEnabled);
}
async function rawlog(base, action){
  await fetch(base + '/rawlog/' + action, {method:'POST'});
  poll();
}
poll();
setInterval(poll, 1500);
</script></body></html>
)HTML";

void handleRoot() {
  webServer.send(200, "text/html", CONTROL_PAGE);
}

void webUiBegin() {
  String host = String("bb-") + (I_AM_LEFT ? "left" : "right");
  if (!MDNS.begin(host.c_str())) {
    Serial.println(F("mDNS init failed (use the IP printed above instead)"));
  } else {
    Serial.printf("mDNS ready: http://%s.local/\n", host.c_str());
  }

  webServer.on("/state", HTTP_GET, handleState);
  if (I_AM_LEFT) {
    webServer.on("/", HTTP_GET, handleRoot);
    webServer.on("/toggle", HTTP_POST, handleToggle);
  }
  rawLogRegisterRoutes();
  webServer.begin();
}

void webUiHandleClient() {
  webServer.handleClient();
}
