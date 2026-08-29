// RawLog.ino
// Short dedicated raw-IMU capture sessions to onboard flash (LittleFS), for offline
// threshold tuning. Runs independently on each board. Not intended for continuous use
// during real research trials -- see docs/superpowers/specs/2026-08-29-motor-toggle-webapp-design.md.

#include <FS.h>
#include <LittleFS.h>

#define RAWLOG_PATH        "/rawlog.csv"
#define RAWLOG_MAX_MS       180000UL   // 3 min safety cap
#define RAWLOG_MAX_BYTES    2000000UL  // ~2MB safety cap

static bool     g_rawLogActive = false;
static File     g_rawLogFile;
static uint32_t g_rawLogStartMs = 0;
static size_t   g_rawLogBytes = 0;

bool rawLogActive() { return g_rawLogActive; }
size_t rawLogBytes() { return g_rawLogBytes; }

void rawLogBegin() {
  if (!LittleFS.begin(true)) {
    Serial.println(F("LittleFS mount failed -- raw logging unavailable"));
  }
}

static void rawLogStop() {
  if (g_rawLogActive) {
    g_rawLogFile.close();
    g_rawLogActive = false;
  }
}

void rawLogTick() {
  if (!g_rawLogActive) return;

  uint32_t elapsed = millis() - g_rawLogStartMs;
  if (elapsed > RAWLOG_MAX_MS || g_rawLogBytes > RAWLOG_MAX_BYTES) {
    rawLogStop();
    return;
  }

  Sample s = readSample();
  String row = String(millis()) + "," +
               String(s.ax, 4) + "," + String(s.ay, 4) + "," + String(s.az, 4) + "," +
               String(s.gx, 4) + "," + String(s.gy, 4) + "," + String(s.gz, 4) + "," +
               String(s.tempC, 2) + "\n";
  g_rawLogFile.print(row);
  g_rawLogBytes += row.length();
}

void handleRawLogStart() {
  rawLogStop(); // in case already running, restart cleanly
  g_rawLogFile = LittleFS.open(RAWLOG_PATH, "w");
  g_rawLogFile.println("ts_ms,ax,ay,az,gx,gy,gz,tempC");
  g_rawLogBytes = 0;
  g_rawLogStartMs = millis();
  g_rawLogActive = true;
  webServer.send(200, "application/json", "{\"rawLogging\":true}");
}

void handleRawLogStop() {
  rawLogStop();
  webServer.send(200, "application/json", "{\"rawLogging\":false}");
}

void handleRawLogDownload() {
  File f = LittleFS.open(RAWLOG_PATH, "r");
  if (!f) {
    webServer.send(404, "text/plain", "no log file");
    return;
  }
  webServer.sendHeader("Access-Control-Allow-Origin", "*");
  webServer.streamFile(f, "text/csv");
  f.close();
}

void handleRawLogClear() {
  if (g_rawLogActive) {
    webServer.send(409, "text/plain", "stop logging before clearing");
    return;
  }
  LittleFS.remove(RAWLOG_PATH);
  g_rawLogBytes = 0;
  webServer.send(200, "application/json", "{\"cleared\":true}");
}

void rawLogRegisterRoutes() {
  webServer.on("/rawlog/start",    HTTP_POST, handleRawLogStart);
  webServer.on("/rawlog/stop",     HTTP_POST, handleRawLogStop);
  webServer.on("/rawlog/download", HTTP_GET,  handleRawLogDownload);
  webServer.on("/rawlog/clear",    HTTP_POST, handleRawLogClear);
}
