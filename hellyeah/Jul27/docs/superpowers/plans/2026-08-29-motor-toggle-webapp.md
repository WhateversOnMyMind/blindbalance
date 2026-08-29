# Motor Toggle & Status Web App Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a phone-reachable control page hosted on the LEFT board that toggles the vibration motor on both boards together, shows live status for both arms, and logs each toggle to the existing Google Sheet.

**Architecture:** Both boards run a `WebServer` (ESP32 Arduino core) with a unique mDNS hostname and a `GET /state` JSON endpoint. Motor mode piggybacks on the existing ESP-NOW `PeerMsg` broadcast (already sent every 2s) with the LEFT board as sole authority — the RIGHT board only ever adopts a value it received from a packet tagged `'L'`. The LEFT board additionally serves `GET /` (the control page) and `POST /toggle`, which flips its local motor flag and fires one HTTP POST to the existing Google Apps Script as an analyzable marker.

**Tech Stack:** ESP32 Arduino core `WebServer.h`, `ESPmDNS.h`; existing `WiFi.h`, `HTTPClient.h`, `esp_now.h` already in the sketch.

**Adaptation note on testing:** This is embedded firmware talking to real SPI/IMU hardware and two physical boards over ESP-NOW — there is no host-side unit test framework in this repo, and `arduino-cli` is not installed in this environment, so compiling and flashing is a manual step you do in the Arduino IDE. Flashing an ESP32 takes real time (reboot, reconnect WiFi, re-pair ESP-NOW), so this plan batches verification into a small number of full flash-and-check cycles rather than one per code change, and spells out exact `curl` commands / expected Serial output / expected browser behavior for each checkpoint. Code-writing steps stay small; hardware-verification steps are the checkpoints.

---

## File Structure

Arduino IDE treats every `.ino` file in the sketch folder as one tab, concatenates them (main sketch file first, others alphabetically) into a single compilation unit, and auto-generates function prototypes — so globals defined in `Jul27.ino` are visible from other tabs with no `extern` needed, as long as `Jul27.ino`'s definitions come before any tab that uses them (guaranteed, since it's always concatenated first).

- **Modify:** `Jul27.ino` — add the `g_motorEnabled` flag, extend `PeerMsg`, gate the motor output, wire the new tab's `begin()`/`handleClient()` calls into `setup()`/`loop()`.
- **Create:** `WebUI.ino` — new tab: `WebServer` + mDNS setup, `/state`, `/toggle`, and the control page HTML+JS.

No new libraries beyond what ships with the ESP32 Arduino core.

---

### Task 1: Motor-enable flag, PeerMsg sync, and output gating

**Files:**
- Modify: `Jul27.ino:99-110` (arm state globals)
- Modify: `Jul27.ino:117-121` (`PeerMsg` struct)
- Modify: `Jul27.ino:277-289` (`onRecv`)
- Modify: `Jul27.ino:305-342` (`feedbackTick`)

- [ ] **Step 1: Add the motor-enable global**

In `Jul27.ino`, in the "this board's arm state" block (around line 109, right after `uint32_t g_lastSwingTime = 0;`), add:

```cpp
volatile bool g_motorEnabled = true;   // LEFT sets via /toggle; RIGHT adopts from peer packets
```

- [ ] **Step 2: Extend `PeerMsg` with the motor flag**

Replace the existing struct (lines 117-121):

```cpp
typedef struct __attribute__((packed)) {
  char     arm;        // 'L' or 'R'
  uint32_t count;      // peer's last feedback-window swing count
  float    distance;   // peer's last feedback-window arc length (m)
} PeerMsg;
```

with:

```cpp
typedef struct __attribute__((packed)) {
  char     arm;          // 'L' or 'R'
  uint32_t count;        // peer's last feedback-window swing count
  float    distance;     // peer's last feedback-window arc length (m)
  bool     motorEnabled; // sender's motor mode (only 'L' is authoritative)
} PeerMsg;
```

- [ ] **Step 3: Adopt the LEFT board's motor mode in `onRecv`**

In `onRecv` (lines 277-289), after the existing `peerRxMs = millis();` line, add:

```cpp
  if (!I_AM_LEFT && m.arm == 'L') g_motorEnabled = m.motorEnabled;
```

So the full function body reads:

```cpp
  if (len != sizeof(PeerMsg)) return;
  PeerMsg m; memcpy(&m, data, sizeof(m));
  if (m.arm == ARM_CHAR) return;         // ignore my own / same-arm packets
  peerArm   = m.arm;
  peerCount = m.count;
  peerDist  = m.distance;
  peerRxMs  = millis();
  if (!I_AM_LEFT && m.arm == 'L') g_motorEnabled = m.motorEnabled;
```

- [ ] **Step 4: Include the flag in the outgoing broadcast**

In `feedbackTick` (line 311), replace:

```cpp
  PeerMsg m; m.arm = ARM_CHAR; m.count = myC; m.distance = myD;
```

with:

```cpp
  PeerMsg m; m.arm = ARM_CHAR; m.count = myC; m.distance = myD; m.motorEnabled = g_motorEnabled;
```

- [ ] **Step 5: Gate the vibration output on the flag**

In `feedbackTick` (line 341), replace:

```cpp
  analogWrite(PIN_VIBE, pwm);
```

with:

```cpp
  analogWrite(PIN_VIBE, g_motorEnabled ? pwm : 0);
```

(The two earlier `analogWrite(PIN_VIBE, 0)` early-returns in the same function — for "no peer" and "balanced/no data" cases — are already off and need no change.)

- [ ] **Step 6: Compile-check and flash both boards (regression baseline)**

Since `g_motorEnabled` defaults to `true`, behavior should be byte-for-byte identical to before this task. In the Arduino IDE: Verify (compile) with no errors, then flash both the LEFT and RIGHT board as you normally would.

Expected Serial output on both boards: identical boot sequence to before (IMU found, WiFi connected, ESP-NOW ready, calibration, then swing/feedback logging as usual). Swing the arms and confirm the motor still buzzes on the lagging arm exactly as before this change — this confirms the new field didn't break ESP-NOW deserialization or the feedback loop.

- [ ] **Step 7: Commit**

```bash
git add Jul27.ino
git commit -m "Add motor-enable flag synced over ESP-NOW, gate vibration output on it"
```

---

### Task 2: Web server tab — status endpoint

**Files:**
- Create: `WebUI.ino`
- Modify: `Jul27.ino` (`setup()`/`loop()` wiring)

- [ ] **Step 1: Create `WebUI.ino` with the server instance and `/state`**

```cpp
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
```

- [ ] **Step 2: Wire it into `Jul27.ino`**

In `setup()`, after `initEspNow();` (line 359) and before `calibrateGyro();` (line 361), add:

```cpp
  webUiBegin();                  // start local web server + mDNS
```

In `loop()`, right after `updateArm(dt);` (line 377), add:

```cpp
  webUiHandleClient();           // service any pending HTTP requests
```

- [ ] **Step 3: Compile-check and flash both boards**

Verify (compile) with no errors — Arduino IDE will show `WebUI.ino` as a second tab automatically. Flash both boards.

Expected Serial output on each board, after "WiFi connected!": a line like `mDNS ready: http://bb-left.local/` (or `bb-right.local` on the other board).

- [ ] **Step 4: Verify `/state` from a machine on the same WiFi network**

```bash
curl http://bb-left.local/state
```

Expected: JSON like `{"arm":"L","motorEnabled":true,"peerFresh":true,"tempC":27.3,"uptimeMs":123456}` (adjust hostname/IP if mDNS doesn't resolve from your machine — use the IP printed over Serial instead, e.g. `curl http://192.168.1.42/state`).

Repeat for `bb-right.local` (or its IP) and confirm `"arm":"R"`.

- [ ] **Step 5: Commit**

```bash
git add WebUI.ino Jul27.ino
git commit -m "Add per-board web server with mDNS hostname and /state status endpoint"
```

---

### Task 3: Toggle endpoint + Sheets marker logging (LEFT only)

**Files:**
- Modify: `WebUI.ino`

- [ ] **Step 1: Add the Sheets marker POST helper**

In `WebUI.ino`, add (this reuses the same `scriptURL` already defined in `Jul27.ino`):

```cpp
static void postToggleEvent(bool state) {
  if (WiFi.status() != WL_CONNECTED) return;
  HTTPClient http;
  http.begin(scriptURL);
  http.addHeader("Content-Type", "application/json");
  String json = "{\"event\":\"motor_toggle\",\"state\":\"" +
                String(state ? "on" : "off") + "\",\"ts\":" + String(millis()) + "}";
  http.POST(json);
  http.end();
}
```

- [ ] **Step 2: Add the `/toggle` handler**

```cpp
void handleToggle() {
  g_motorEnabled = !g_motorEnabled;
  postToggleEvent(g_motorEnabled);
  webServer.sendHeader("Access-Control-Allow-Origin", "*");
  webServer.send(200, "application/json", jsonState());
}
```

- [ ] **Step 3: Register it only on the LEFT board**

In `webUiBegin()`, replace:

```cpp
  webServer.on("/state", HTTP_GET, handleState);
  webServer.begin();
```

with:

```cpp
  webServer.on("/state", HTTP_GET, handleState);
  if (I_AM_LEFT) {
    webServer.on("/toggle", HTTP_POST, handleToggle);
  }
  webServer.begin();
```

- [ ] **Step 4: Compile-check and flash the LEFT board**

Only the LEFT board's code path changed behavior (RIGHT still compiles the same handler function but never registers the route — that's fine, no need to reflash RIGHT for this task).

- [ ] **Step 5: Verify the toggle end-to-end**

```bash
curl -X POST http://bb-left.local/toggle
```

Expected: JSON response with `"motorEnabled":false` (flipped from the default `true`). Run it again and confirm it flips back to `"motorEnabled":true`.

Swing-test: with the motor toggled off, physically create an imbalance between the arms (or check Serial) and confirm the LEFT board's motor does not buzz even when the feedback calc would otherwise drive it, and that the RIGHT board's motor also stops within about 2 seconds (one `PeerMsg` broadcast cycle) — confirms the sync, not just the local flag.

Check the Google Sheet for a new row matching `motor_toggle` with a sane timestamp for each `curl` call above.

- [ ] **Step 6: Commit**

```bash
git add WebUI.ino
git commit -m "Add /toggle endpoint on the LEFT board with Sheets marker logging"
```

---

### Task 4: Control page (LEFT only)

**Files:**
- Modify: `WebUI.ino`

- [ ] **Step 1: Add the page markup as a PROGMEM string**

```cpp
static const char CONTROL_PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head><meta name="viewport" content="width=device-width, initial-scale=1">
<title>BlindBalance Control</title>
<style>
body{font-family:sans-serif;text-align:center;padding:1em;background:#111;color:#eee}
button{font-size:1.5em;padding:0.6em 1.2em;border-radius:0.5em;border:none}
.on{background:#2ecc71}.off{background:#e74c3c}
.tile{margin:1em auto;padding:0.8em;border-radius:0.5em;background:#222;max-width:20em}
.stale{opacity:0.4}
</style></head><body>
<h2>Motor Feedback</h2>
<button id="toggleBtn" onclick="toggle()">...</button>
<div class="tile" id="tileL"><b>Left</b><div id="infoL">loading...</div></div>
<div class="tile" id="tileR"><b>Right</b><div id="infoR">loading...</div></div>
<script>
async function poll(){
  try{
    const l = await (await fetch('/state')).json();
    render(l, 'infoL', 'tileL');
    setBtn(l.motorEnabled);
  }catch(e){}
  try{
    const r = await (await fetch('http://bb-right.local/state')).json();
    render(r, 'infoR', 'tileR');
  }catch(e){ document.getElementById('tileR').classList.add('stale'); }
}
function render(s, infoId, tileId){
  document.getElementById(infoId).innerText =
    'motor: ' + (s.motorEnabled ? 'on' : 'off') +
    ' | peer: ' + (s.peerFresh ? 'ok' : 'stale') +
    ' | temp: ' + s.tempC + 'C';
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
poll();
setInterval(poll, 1500);
</script></body></html>
)HTML";

void handleRoot() {
  webServer.send(200, "text/html", CONTROL_PAGE);
}
```

- [ ] **Step 2: Register the route on LEFT only**

In `webUiBegin()`, replace:

```cpp
  if (I_AM_LEFT) {
    webServer.on("/toggle", HTTP_POST, handleToggle);
  }
```

with:

```cpp
  if (I_AM_LEFT) {
    webServer.on("/", HTTP_GET, handleRoot);
    webServer.on("/toggle", HTTP_POST, handleToggle);
  }
```

- [ ] **Step 3: Compile-check and flash the LEFT board**

- [ ] **Step 4: Verify in a phone browser on the same WiFi network**

Open `http://bb-left.local/` (or the LEFT board's IP from Serial if mDNS doesn't resolve on your phone). Confirm:
- The page loads with a colored ON/OFF button and two tiles (Left, Right).
- Both tiles show `motor:`, `peer:`, and `temp:` values within ~2 seconds of load.
- Tapping the button flips its color/label immediately, and within ~2 seconds the Right tile's `motor:` value matches it.
- Power off the RIGHT board and confirm its tile fades (gets the `stale` look) rather than freezing on stale data forever.

- [ ] **Step 5: Commit**

```bash
git add WebUI.ino
git commit -m "Add mobile control page with live status tiles for both arms"
```

---

## Plan Self-Review Notes

- **Spec coverage:** motor toggle (Task 1, 3), both-boards-together sync (Task 1 step 3-4), status panel for both arms (Task 2, 4), Sheets marker logging (Task 3) are all covered by this plan. Raw IMU logging and drift mitigation are intentionally deferred to their own follow-on plans, per the scope split described above.
- **Type/name consistency:** `g_motorEnabled`, `jsonState()`, `webUiBegin()`, `webUiHandleClient()`, `CONTROL_PAGE`, `handleToggle`/`handleState`/`handleRoot` are used consistently across all four tasks.
- **No placeholders:** every step above shows complete, copy-pasteable code and exact verification commands/expected output.
