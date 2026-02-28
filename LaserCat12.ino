/*
  LaserCat ESP8266 (Wemos D1 mini) - Arduino IDE
  Version: Custom Picker (B) for:
   - Bewegung + Bereich & Anti-Ecken (Number-Picker Modal)
   - Wochenplan Start/Stop (Time-Picker Modal)
  Erweiterung:
   - Pro Tag & Zeitfenster (A/B) ein Preset zuordnen (Custom/Sanft/Normal/Wild/Smart)
   - /api/state zeigt aktives Preset (Tag/Fenster/Id/Name)
   - UI zeigt aktives Preset in Pills/Preview
   - NEU: Button "Fenster aus" / "Fenster an" (aktuelles Zeitfenster runtime blockieren)
   - NEU: Save-Blockierung in der UI:
        * Stop darf nicht <= Start sein
        * Fenster A und B dürfen sich nicht überlappen

  OTA Passwort: Variable OTA_PASSWORD
  DEVICE_NAME: Variable (für OTA + Default MQTT_BASE + UI)

  WICHTIG:
  - Servos separate 5V Versorgung (>=2A), GND gemeinsam mit Wemos!
  - Laser 5V besser über Transistor/MOSFET an PIN_LASER schalten.
*/

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <LittleFS.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>   // ArduinoJson 7.x
#include <Servo.h>
#include <time.h>
#include <ArduinoOTA.h>
#include <algorithm> // std::swap

// ✅ WICHTIG: Types in types.h auslagern, damit Arduino Auto-Prototyping nicht kaputtgeht.
#include "types.h"

// ===== Gerätename (für OTA + Default MQTT Base + UI) =====
const char* DEVICE_NAME = "LaserCat";

// ===== OTA Passwort (leer = kein Passwort) =====
const char* OTA_PASSWORD = "hier Dein Sicheres Passwort";  // "" => kein Passwort

// ===== Pins (avoid boot strap pins D3/D4/D8) =====
static const uint8_t PIN_SERVO_PAN  = D5;   // GPIO14
static const uint8_t PIN_SERVO_TILT = D6;   // GPIO12
static const uint8_t PIN_LASER      = D7;   // GPIO13 -> transistor/MOSFET recommended

// ===== WiFi / MQTT =====
const char* WIFI_SSID = "SSID";
const char* WIFI_PASS = "PASS";

const char* MQTT_HOST = "HOST";
const uint16_t MQTT_PORT = PORT;
const char* MQTT_USER = "USER";
const char* MQTT_PASS = "PASS";

// >>> MQTT BASE TOPIC (editable) <<<
String MQTT_BASE = String(DEVICE_NAME);

// ===== Config instance =====
Config cfg;

// ===== Globals =====
ESP8266WebServer server(80);
WiFiClient espClient;
PubSubClient mqtt(espClient);

Servo servoPan, servoTilt;
bool servosAttached = false;

const char* CFG_PATH = "/config.json";

// runtime state for motion
float curPan = 90, curTilt = 90;
float segStartPan = 90, segStartTilt = 90, segTargetPan = 90, segTargetTilt = 90;
uint32_t segStartMs = 0;
uint32_t segDurMs = 1500;
uint32_t dwellUntilMs = 0;

// Session/Cooldown cycle state (within schedule)
uint32_t cycleSessionStartMs = 0;
uint32_t cycleCooldownUntilMs = 0;

bool wasActiveLastLoop = false;

// Smart mode extra state
int smartBurstRemaining = 0;
float smartDirPan = 0.f, smartDirTilt = 0.f;
int smartDwellOverrideMin = -1;
int smartDwellOverrideMax = -1;

// ---- Preset runtime override ----
struct MotionOverride {
  bool active = false;
  float speedDegPerSec = 0;
  uint16_t dwellMin = 0;
  uint16_t dwellMax = 0;
  MotionMode mode = MODE_RANDOM;
};
MotionOverride mo;

int8_t activeDay = -1;        // 0..6 or -1
char activeWin = 0;           // 'A'/'B' or 0
uint8_t activePreset = 0;     // PresetId

// ---- Manual window block (runtime only; resets on reboot) ----
bool windowBlocked = false;
int8_t blockedDay = -1;   // 0..6
char blockedWin = 0;      // 'A'/'B'

// ===== Utils =====
static int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static uint32_t msNow() { return millis(); }

static float smoothstep(float t) {
  t = clampf(t, 0.f, 1.f);
  return t * t * (3.f - 2.f * t);
}

String topic(const char* suffix) { return MQTT_BASE + "/" + String(suffix); }

void laserOn(bool on) { digitalWrite(PIN_LASER, on ? HIGH : LOW); }

void writeServos(float pan, float tilt) {
  servoPan.write((int)pan);
  servoTilt.write((int)tilt);
}

// ===== Servo attach/detach =====
void ensureServosAttached() {
  if (servosAttached) return;
  servoPan.attach(PIN_SERVO_PAN);
  servoTilt.attach(PIN_SERVO_TILT);
  servosAttached = true;
}

void detachServos() {
  if (!servosAttached) return;
  servoPan.detach();
  servoTilt.detach();
  servosAttached = false;
}

// ===== Time =====
bool timeIsSet() {
  time_t now = time(nullptr);
  return now > 100000;
}

void setupTime() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/3", 1);
  tzset();
}

String localTimeString() {
  if (!timeIsSet()) return "";
  time_t now = time(nullptr);
  struct tm tt;
  localtime_r(&now, &tt);
  char buf[32];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
           tt.tm_year + 1900, tt.tm_mon + 1, tt.tm_mday, tt.tm_hour, tt.tm_min, tt.tm_sec);
  return String(buf);
}

// ===== Bounds with edge margin =====
int safePanMin()  { return clampi(cfg.panMin  + cfg.edgeMarginDeg, 0, 180); }
int safePanMax()  { return clampi(cfg.panMax  - cfg.edgeMarginDeg, 0, 180); }
int safeTiltMin() { return clampi(cfg.tiltMin + cfg.edgeMarginDeg, 0, 180); }
int safeTiltMax() { return clampi(cfg.tiltMax - cfg.edgeMarginDeg, 0, 180); }

void normalizeBounds() {
  cfg.panMin  = clampi(cfg.panMin,  0, 180);
  cfg.panMax  = clampi(cfg.panMax,  0, 180);
  cfg.tiltMin = clampi(cfg.tiltMin, 0, 180);
  cfg.tiltMax = clampi(cfg.tiltMax, 0, 180);
  if (cfg.panMin > cfg.panMax)   std::swap(cfg.panMin, cfg.panMax);
  if (cfg.tiltMin > cfg.tiltMax) std::swap(cfg.tiltMin, cfg.tiltMax);

  cfg.edgeMarginDeg = clampi(cfg.edgeMarginDeg, 0, 30);

  if (safePanMin()  >= safePanMax())  cfg.edgeMarginDeg = 0;
  if (safeTiltMin() >= safeTiltMax()) cfg.edgeMarginDeg = 0;
}

// ===== Schedule gate =====
bool inWindow(const TimeWindow& w, int mins) {
  if (!w.enabled) return false;
  return (mins >= (int)w.startMin) && (mins < (int)w.stopMin);
}

bool isBlocked(int day, char win) {
  return windowBlocked && (day == blockedDay) && (win == blockedWin);
}

// forward decl
bool getActiveWindowNow(int &dayOut, char &winOut, uint8_t &presetOut);

void clearBlockIfExpired() {
  if (!windowBlocked) return;

  int d; char w; uint8_t p;
  bool ok = getActiveWindowNow(d, w, p);
  if (!ok) { windowBlocked = false; blockedDay = -1; blockedWin = 0; return; }

  if (d != blockedDay || w != blockedWin) {
    windowBlocked = false; blockedDay = -1; blockedWin = 0;
  }
}

bool allowedByScheduleNow() {
  if (!timeIsSet()) return false;

  time_t now = time(nullptr);
  struct tm t;
  localtime_r(&now, &t);

  int wday = t.tm_wday;  // 0..6 (So..Sa)
  int mins = t.tm_hour * 60 + t.tm_min;

  DaySchedule2& d = cfg.days[wday];
  if (!d.dayEnabled) return false;

  bool aOk = inWindow(d.a, mins) && !isBlocked(wday, 'A');
  bool bOk = inWindow(d.b, mins) && !isBlocked(wday, 'B');
  return aOk || bOk;
}

// ===== Preset -> runtime override =====
void setOverrideFromPreset(uint8_t p) {
  mo.active = (p != PRESET_CUSTOM);
  if (!mo.active) return;

  if (p == PRESET_SOFT) {
    mo.speedDegPerSec = 45;
    mo.dwellMin = 350;
    mo.dwellMax = 1200;
    mo.mode = MODE_RANDOM;
  } else if (p == PRESET_NORMAL) {
    mo.speedDegPerSec = 75;
    mo.dwellMin = 150;
    mo.dwellMax = 900;
    mo.mode = MODE_RANDOM;
  } else if (p == PRESET_WILD) {
    mo.speedDegPerSec = 160;
    mo.dwellMin = 40;
    mo.dwellMax = 320;
    mo.mode = MODE_SCAN;
  } else if (p == PRESET_SMART) {
    mo.speedDegPerSec = 95;
    mo.dwellMin = 80;
    mo.dwellMax = 650;
    mo.mode = MODE_SMART;
  } else {
    mo.active = false;
  }
}

float effSpeed()       { return mo.active ? mo.speedDegPerSec : cfg.speedDegPerSec; }
uint16_t effDwellMin() { return mo.active ? mo.dwellMin : cfg.dwellMsMin; }
uint16_t effDwellMax() { return mo.active ? mo.dwellMax : cfg.dwellMsMax; }
MotionMode effMode()   { return mo.active ? mo.mode : cfg.mode; }

bool getActiveWindowNow(int &dayOut, char &winOut, uint8_t &presetOut) {
  dayOut = -1; winOut = 0; presetOut = PRESET_CUSTOM;
  if (!timeIsSet()) return false;

  time_t now = time(nullptr);
  tm t; localtime_r(&now, &t);

  int wday = t.tm_wday;
  int mins = t.tm_hour * 60 + t.tm_min;

  DaySchedule2 &d = cfg.days[wday];
  if (!d.dayEnabled) return false;

  if (inWindow(d.a, mins) && !isBlocked(wday, 'A')) { dayOut = wday; winOut = 'A'; presetOut = d.a.preset; return true; }
  if (inWindow(d.b, mins) && !isBlocked(wday, 'B')) { dayOut = wday; winOut = 'B'; presetOut = d.b.preset; return true; }
  return false;
}

void updatePresetOverride(bool allowImmediate) {
  int d; char w; uint8_t p;
  bool ok = getActiveWindowNow(d, w, p);

  if (!ok) {
    mo.active = false;
    activeDay = -1; activeWin = 0; activePreset = PRESET_CUSTOM;
    return;
  }

  if (d != activeDay || w != activeWin || p != activePreset) {
    activeDay = d; activeWin = w; activePreset = p;
    setOverrideFromPreset(p);

    if (allowImmediate) {
      smartBurstRemaining = 0;
      dwellUntilMs = 0;
    }
  }
}

// ===== Session/Cooldown cycle =====
bool cycleInCooldown() {
  return (cycleCooldownUntilMs != 0) && (msNow() < cycleCooldownUntilMs);
}

bool cycleSessionRunning() {
  if (cfg.sessionMaxMin == 0) return true;
  if (cycleSessionStartMs == 0) return false;
  uint32_t maxMs = (uint32_t)cfg.sessionMaxMin * 60UL * 1000UL;
  return (msNow() - cycleSessionStartMs) < maxMs;
}

void cycleReset() {
  cycleSessionStartMs = 0;
  cycleCooldownUntilMs = 0;
}

void cycleTick(bool schedOk) {
  if (!cfg.runEnabled || !schedOk) {
    cycleReset();
    return;
  }

  if (cfg.sessionMaxMin == 0) {
    cycleSessionStartMs = 1;
    cycleCooldownUntilMs = 0;
    return;
  }

  if (cycleInCooldown()) return;

  if (cycleCooldownUntilMs != 0 && msNow() >= cycleCooldownUntilMs) {
    cycleCooldownUntilMs = 0;
    cycleSessionStartMs = msNow();
    return;
  }

  if (cycleSessionStartMs == 0) {
    cycleSessionStartMs = msNow();
    return;
  }

  if (!cycleSessionRunning()) {
    cycleSessionStartMs = 0;
    if (cfg.cooldownMin > 0) {
      cycleCooldownUntilMs = msNow() + (uint32_t)cfg.cooldownMin * 60UL * 1000UL;
    } else {
      cycleCooldownUntilMs = 0;
      cycleSessionStartMs = msNow();
    }
  }
}

bool isActiveNow() {
  bool schedOk = allowedByScheduleNow();
  cycleTick(schedOk);

  if (!cfg.runEnabled) return false;
  if (!schedOk) return false;
  if (cfg.sessionMaxMin == 0) return true;
  if (cycleInCooldown()) return false;
  return cycleSessionRunning();
}

// ===== Motion =====
float randIn(int lo, int hi) { return (float)random(lo, hi + 1); }

void startSegment(float fromPan, float fromTilt, float toPan, float toTilt, float speedScale = 1.0f) {
  segStartPan = fromPan;
  segStartTilt = fromTilt;
  segTargetPan = toPan;
  segTargetTilt = toTilt;
  segStartMs = msNow();

  float dPan = fabs(toPan - fromPan);
  float dTilt = fabs(toTilt - fromTilt);
  float dist = sqrt(dPan * dPan + dTilt * dTilt);

  float spd = clampf(effSpeed() * speedScale, 5.f, 240.f);
  float seconds = dist / spd;
  segDurMs = (uint32_t)clampf(seconds * 1000.f, 350.f, 6000.f);

  dwellUntilMs = 0;
}

void smartSetDwellOverride(int mn, int mx) {
  smartDwellOverrideMin = mn;
  smartDwellOverrideMax = mx;
}

void chooseNextTarget() {
  int pMin = safePanMin(), pMax = safePanMax();
  int tMin = safeTiltMin(), tMax = safeTiltMax();
  if (pMin >= pMax) { pMin = cfg.panMin; pMax = cfg.panMax; }
  if (tMin >= tMax) { tMin = cfg.tiltMin; tMax = cfg.tiltMax; }

  float toPan = 90, toTilt = 90;
  float speedScale = 1.0f;

  MotionMode mode = effMode();

  if (mode == MODE_RANDOM) {
    toPan = randIn(pMin, pMax);
    toTilt = randIn(tMin, tMax);
  } else if (mode == MODE_ORBIT) {
    float cPan = (pMin + pMax) / 2.0f;
    float cTilt = (tMin + tMax) / 2.0f;
    float rPan = (pMax - pMin) / 2.2f;
    float rTilt = (tMax - tMin) / 2.2f;
    float ang = random(0, 360) * 3.1415926f / 180.f;
    float jitter = random(-15, 16) / 100.0f;
    toPan = cPan + cos(ang) * rPan * (1.0f + jitter);
    toTilt = cTilt + sin(ang) * rTilt * (1.0f + jitter);
    toPan = clampf(toPan, (float)pMin, (float)pMax);
    toTilt = clampf(toTilt, (float)tMin, (float)tMax);
  } else if (mode == MODE_SCAN) {
    bool horiz = random(0, 2) == 0;
    if (horiz) { toPan = (random(0, 2) == 0) ? pMin : pMax; toTilt = randIn(tMin, tMax); }
    else { toTilt = (random(0, 2) == 0) ? tMin : tMax; toPan = randIn(pMin, pMax); }
  } else { // MODE_SMART
    const int pSpan = max(12, (pMax - pMin));
    const int tSpan = max(12, (tMax - tMin));

    if (smartBurstRemaining > 0) {
      smartBurstRemaining--;
      float stepP = smartDirPan * random(6, max(10, pSpan / 8));
      float stepT = smartDirTilt * random(5, max(9, tSpan / 9));
      stepP += random(-4, 5);
      stepT += random(-3, 4);

      toPan = clampf(curPan + stepP, (float)pMin, (float)pMax);
      toTilt = clampf(curTilt + stepT, (float)tMin, (float)tMax);

      speedScale = random(95, 140) / 100.0f;
      smartSetDwellOverride(40, 220);

      startSegment(curPan, curTilt, toPan, toTilt, speedScale);
      return;
    }

    int r = random(0, 100);

    if (r < 14) {
      float midPan = (pMin + pMax) * 0.5f;
      toPan = (curPan < midPan) ? pMax : pMin;
      toTilt = randIn(tMin, tMax);
      speedScale = random(120, 165) / 100.0f;
      smartSetDwellOverride(60, 260);
    } else if (r < 32) {
      float dp = random(-max(8, pSpan / 10), max(8, pSpan / 10) + 1);
      float dt = random(-max(6, tSpan / 12), max(6, tSpan / 12) + 1);

      toPan = clampf(curPan + dp, (float)pMin, (float)pMax);
      toTilt = clampf(curTilt + dt, (float)tMin, (float)tMax);

      speedScale = random(60, 90) / 100.0f;
      smartSetDwellOverride(250, 900);
    } else if (r < 50) {
      float dp = random(-6, 7);
      float dt = random(-5, 6);
      toPan = clampf(curPan + dp, (float)pMin, (float)pMax);
      toTilt = clampf(curTilt + dt, (float)tMin, (float)tMax);
      speedScale = random(55, 85) / 100.0f;
      smartSetDwellOverride(500, 1400);
    } else {
      smartBurstRemaining = random(2, 5);
      smartDirPan = (random(0, 2) == 0) ? -1.f : 1.f;
      smartDirTilt = (random(0, 2) == 0) ? -1.f : 1.f;
      if (random(0, 100) < 35) smartDirPan *= -1.f;

      float stepP = smartDirPan * random(10, max(14, pSpan / 6));
      float stepT = smartDirTilt * random(8, max(12, tSpan / 7));

      toPan = clampf(curPan + stepP, (float)pMin, (float)pMax);
      toTilt = clampf(curTilt + stepT, (float)tMin, (float)tMax);

      speedScale = random(90, 140) / 100.0f;
      smartSetDwellOverride(60, 260);
    }
  }

  startSegment(curPan, curTilt, toPan, toTilt, speedScale);
}

void onArrived() {
  uint16_t mn = effDwellMin();
  uint16_t mx = effDwellMax();

  if (smartDwellOverrideMin >= 0 && smartDwellOverrideMax >= 0) {
    mn = (uint16_t)clampi(smartDwellOverrideMin, 0, 8000);
    mx = (uint16_t)clampi(smartDwellOverrideMax, 0, 8000);
    smartDwellOverrideMin = -1;
    smartDwellOverrideMax = -1;
  }

  if (mn > mx) std::swap(mn, mx);
  uint16_t dwell = (uint16_t)random(mn, mx + 1);
  dwellUntilMs = msNow() + dwell;
}

// ===== Smooth park then detach =====
void parkSmoothThenDetach(uint16_t durationMs = 900) {
  laserOn(false);
  ensureServosAttached();

  float fromPan = curPan;
  float fromTilt = curTilt;
  float toPan  = (float)clampi(cfg.parkPan,  0, 180);
  float toTilt = (float)clampi(cfg.parkTilt, 0, 180);

  uint32_t start = msNow();
  while (msNow() - start < durationMs) {
    float t = (float)(msNow() - start) / (float)durationMs;
    float e = smoothstep(t);
    curPan  = fromPan  + (toPan  - fromPan)  * e;
    curTilt = fromTilt + (toTilt - fromTilt) * e;
    writeServos(curPan, curTilt);
    delay(5);
  }

  curPan = toPan;
  curTilt = toTilt;
  writeServos(curPan, curTilt);

  detachServos();
}

// ===== Config defaults =====
void setDefaultSchedule() {
  for (int i = 0; i < 7; i++) {
    cfg.days[i].dayEnabled = true;
    cfg.days[i].a = { 7 * 60, 8 * 60, false, PRESET_CUSTOM };
    cfg.days[i].b = { 18 * 60, 20 * 60, true,  PRESET_CUSTOM };
  }
  cfg.days[6].b = { 12 * 60, 20 * 60, true, PRESET_CUSTOM };
  cfg.days[0].b = { 12 * 60, 19 * 60, true, PRESET_CUSTOM };
}

// ===== Config persistence =====
bool saveConfig() {
  StaticJsonDocument<4096> doc;

  doc["panMin"] = cfg.panMin;
  doc["panMax"] = cfg.panMax;
  doc["tiltMin"] = cfg.tiltMin;
  doc["tiltMax"] = cfg.tiltMax;
  doc["edgeMarginDeg"] = cfg.edgeMarginDeg;

  doc["parkPan"] = cfg.parkPan;
  doc["parkTilt"] = cfg.parkTilt;

  doc["speed"] = cfg.speedDegPerSec;
  doc["dwellMin"] = cfg.dwellMsMin;
  doc["dwellMax"] = cfg.dwellMsMax;
  doc["laserEnabled"] = cfg.laserEnabled;
  doc["runEnabled"] = cfg.runEnabled;
  doc["mode"] = (int)cfg.mode;

  doc["sessionMaxMin"] = cfg.sessionMaxMin;
  doc["cooldownMin"] = cfg.cooldownMin;

  JsonArray days = doc.createNestedArray("days");
  for (int i = 0; i < 7; i++) {
    JsonObject d = days.createNestedObject();
    d["dayEn"] = cfg.days[i].dayEnabled;

    JsonObject a = d.createNestedObject("a");
    a["en"] = cfg.days[i].a.enabled;
    a["start"] = cfg.days[i].a.startMin;
    a["stop"] = cfg.days[i].a.stopMin;
    a["preset"] = cfg.days[i].a.preset;

    JsonObject b = d.createNestedObject("b");
    b["en"] = cfg.days[i].b.enabled;
    b["start"] = cfg.days[i].b.startMin;
    b["stop"] = cfg.days[i].b.stopMin;
    b["preset"] = cfg.days[i].b.preset;
  }

  File f = LittleFS.open(CFG_PATH, "w");
  if (!f) return false;
  serializeJson(doc, f);
  f.close();
  return true;
}

bool loadConfig() {
  if (!LittleFS.exists(CFG_PATH)) return false;
  File f = LittleFS.open(CFG_PATH, "r");
  if (!f) return false;

  StaticJsonDocument<4096> doc;
  auto err = deserializeJson(doc, f);
  f.close();
  if (err) return false;

  cfg.panMin = doc["panMin"] | cfg.panMin;
  cfg.panMax = doc["panMax"] | cfg.panMax;
  cfg.tiltMin = doc["tiltMin"] | cfg.tiltMin;
  cfg.tiltMax = doc["tiltMax"] | cfg.tiltMax;
  cfg.edgeMarginDeg = doc["edgeMarginDeg"] | cfg.edgeMarginDeg;

  cfg.parkPan = doc["parkPan"] | cfg.parkPan;
  cfg.parkTilt = doc["parkTilt"] | cfg.parkTilt;

  cfg.speedDegPerSec = doc["speed"] | cfg.speedDegPerSec;
  cfg.dwellMsMin = doc["dwellMin"] | cfg.dwellMsMin;
  cfg.dwellMsMax = doc["dwellMax"] | cfg.dwellMsMax;
  cfg.laserEnabled = doc["laserEnabled"] | cfg.laserEnabled;
  cfg.runEnabled = doc["runEnabled"] | cfg.runEnabled;

  int m = doc["mode"] | (int)cfg.mode;
  cfg.mode = (MotionMode)clampi(m, 0, 3);

  cfg.sessionMaxMin = doc["sessionMaxMin"] | cfg.sessionMaxMin;
  cfg.cooldownMin = doc["cooldownMin"] | cfg.cooldownMin;

  JsonArray days = doc["days"].as<JsonArray>();
  if (days && days.size() == 7) {
    for (int i = 0; i < 7; i++) {
      cfg.days[i].dayEnabled = days[i]["dayEn"] | cfg.days[i].dayEnabled;

      JsonObject a = days[i]["a"];
      JsonObject b = days[i]["b"];
      if (a) {
        cfg.days[i].a.enabled   = a["en"] | cfg.days[i].a.enabled;
        cfg.days[i].a.startMin  = a["start"] | cfg.days[i].a.startMin;
        cfg.days[i].a.stopMin   = a["stop"] | cfg.days[i].a.stopMin;
        cfg.days[i].a.preset    = (uint8_t)clampi(a["preset"] | cfg.days[i].a.preset, 0, 4);
      }
      if (b) {
        cfg.days[i].b.enabled   = b["en"] | cfg.days[i].b.enabled;
        cfg.days[i].b.startMin  = b["start"] | cfg.days[i].b.startMin;
        cfg.days[i].b.stopMin   = b["stop"] | cfg.days[i].b.stopMin;
        cfg.days[i].b.preset    = (uint8_t)clampi(b["preset"] | cfg.days[i].b.preset, 0, 4);
      }
    }
  }

  normalizeBounds();
  if (cfg.dwellMsMin > cfg.dwellMsMax) std::swap(cfg.dwellMsMin, cfg.dwellMsMax);
  return true;
}

// ===== Web JSON config (for UI) =====
String configAsJson() {
  StaticJsonDocument<4096> doc;

  doc["panMin"] = cfg.panMin;
  doc["panMax"] = cfg.panMax;
  doc["tiltMin"] = cfg.tiltMin;
  doc["tiltMax"] = cfg.tiltMax;
  doc["edgeMarginDeg"] = cfg.edgeMarginDeg;

  doc["parkPan"] = cfg.parkPan;
  doc["parkTilt"] = cfg.parkTilt;

  doc["speed"] = cfg.speedDegPerSec;
  doc["dwellMin"] = cfg.dwellMsMin;
  doc["dwellMax"] = cfg.dwellMsMax;
  doc["laserEnabled"] = cfg.laserEnabled;
  doc["runEnabled"] = cfg.runEnabled;
  doc["mode"] = (int)cfg.mode;

  doc["sessionMaxMin"] = cfg.sessionMaxMin;
  doc["cooldownMin"] = cfg.cooldownMin;

  JsonArray days = doc.createNestedArray("days");
  for (int i = 0; i < 7; i++) {
    JsonObject d = days.createNestedObject();
    d["dayEn"] = cfg.days[i].dayEnabled;

    JsonObject a = d.createNestedObject("a");
    a["en"] = cfg.days[i].a.enabled;
    a["start"] = cfg.days[i].a.startMin;
    a["stop"] = cfg.days[i].a.stopMin;
    a["preset"] = cfg.days[i].a.preset;

    JsonObject b = d.createNestedObject("b");
    b["en"] = cfg.days[i].b.enabled;
    b["start"] = cfg.days[i].b.startMin;
    b["stop"] = cfg.days[i].b.stopMin;
    b["preset"] = cfg.days[i].b.preset;
  }

  String out;
  serializeJson(doc, out);
  return out;
}

// ===== MQTT publish helpers =====
void mqttPublishState(bool retained = true) {
  if (!mqtt.connected()) return;

  mqtt.publish(topic("state/run").c_str(), (cfg.runEnabled ? "ON" : "OFF"), retained);

  const bool active = isActiveNow();
  const bool laserOut = active && cfg.laserEnabled;
  mqtt.publish(topic("state/laser").c_str(), (laserOut ? "ON" : "OFF"), retained);
}

// ===== MQTT subscribe =====
void mqttSubscribeAll() { mqtt.subscribe(topic("cmd/run").c_str()); }

bool parseBoolLike(const String& p) {
  return (p == "1" || p == "true" || p == "TRUE" || p == "on" || p == "ON");
}

// (optional) MQTT schedule cmd handler (times only)
void applyScheduleCommand(const String& fullTopic, const String& payload) {
  String prefix = MQTT_BASE + "/";
  if (!fullTopic.startsWith(prefix)) return;
  String rest = fullTopic.substring(prefix.length());

  int p1 = rest.indexOf('/');
  if (p1 < 0) return;
  if (rest.substring(0, p1) != "cmd") return;

  int p2 = rest.indexOf('/', p1 + 1);
  if (p2 < 0) return;
  if (rest.substring(p1 + 1, p2) != "day") return;

  int p3 = rest.indexOf('/', p2 + 1);
  if (p3 < 0) return;
  int day = rest.substring(p2 + 1, p3).toInt();
  if (day < 0 || day > 6) return;

  String tail = rest.substring(p3 + 1);

  if (tail == "dayEn") {
    cfg.days[day].dayEnabled = parseBoolLike(payload);
    saveConfig();
    mqttPublishState(true);
    return;
  }

  int p4 = tail.indexOf('/');
  if (p4 < 0) return;
  String which = tail.substring(0, p4);
  String field = tail.substring(p4 + 1);

  TimeWindow* w = nullptr;
  if (which == "a") w = &cfg.days[day].a;
  else if (which == "b") w = &cfg.days[day].b;
  else return;

  if (field == "en") {
    w->enabled = parseBoolLike(payload);
  } else if (field == "start") {
    w->startMin = (uint16_t)clampi(payload.toInt(), 0, 24 * 60 - 1);
  } else if (field == "stop") {
    w->stopMin = (uint16_t)clampi(payload.toInt(), 0, 24 * 60);
  } else {
    return;
  }

  saveConfig();
  mqttPublishState(true);
}

// ===== MQTT callback =====
void mqttCallback(char* tpc, byte* payload, unsigned int len) {
  String t(tpc);
  String p; p.reserve(len);
  for (unsigned int i = 0; i < len; i++) p += (char)payload[i];
  p.trim();

  if (t == topic("cmd/run")) {
    cfg.runEnabled = (p == "ON" || p == "1" || p == "true" || p == "TRUE" || p == "on");
    cycleReset();
    saveConfig();
    mqttPublishState(true);
  } else {
    applyScheduleCommand(t, p);
  }
}

void mqttEnsure() {
  if (mqtt.connected()) return;
  if (WiFi.status() != WL_CONNECTED) return;

  String clientId = String(DEVICE_NAME) + "-" + String(ESP.getChipId(), HEX);
  if (mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASS)) {
    mqtt.setCallback(mqttCallback);
    mqttSubscribeAll();
    mqttPublishState(true);
  }
}

// ===== Web UI (HTML) =====
const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="de">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>LaserCat</title>
  <style>
    :root{--bg:#0b0f1a;--stroke:rgba(255,255,255,.14);--text:rgba(255,255,255,.92);--muted:rgba(255,255,255,.66);--shadow:0 12px 40px rgba(0,0,0,.35);--r:18px;}
    *{box-sizing:border-box}
    body{margin:0;font-family:ui-sans-serif,system-ui,-apple-system,Segoe UI,Roboto,Helvetica,Arial;
      background:radial-gradient(1200px 700px at 20% -10%, rgba(120,180,255,.25), transparent 55%),
      radial-gradient(900px 600px at 100% 0%, rgba(180,120,255,.18), transparent 60%),
      radial-gradient(900px 700px at 10% 100%, rgba(120,255,200,.12), transparent 55%),var(--bg);
      color:var(--text);}
    .wrap{max-width:1020px;margin:0 auto;padding:18px 14px 50px}
    .topbar{position:sticky;top:0;z-index:9;backdrop-filter:blur(10px);
      background:linear-gradient(to bottom, rgba(11,15,26,.80), rgba(11,15,26,.35));
      border-bottom:1px solid rgba(255,255,255,.08);}
    .topbar-inner{max-width:1020px;margin:0 auto;padding:14px 14px}
    .title-row{display:flex;gap:12px;align-items:center;justify-content:space-between}
    .title{display:flex;flex-direction:column;gap:4px;}
    .title h1{margin:0;font-size:18px;letter-spacing:.3px}
    .sub{color:var(--muted);font-size:12.5px;line-height:1.35}
    .pill{display:inline-flex;align-items:center;gap:8px;padding:7px 10px;border-radius:999px;border:1px solid var(--stroke);
      background:rgba(255,255,255,.06);font-size:12.5px;color:var(--muted);white-space:nowrap;}
    .pill strong{color:var(--text);font-weight:650}
    .pill.ok{border-color:rgba(90,255,170,.35);background:rgba(90,255,170,.10)}
    .pill.warn{border-color:rgba(255,200,90,.35);background:rgba(255,200,90,.10)}
    .pill.bad{border-color:rgba(255,90,90,.35);background:rgba(255,90,90,.10)}
    .grid{display:grid;gap:14px;grid-template-columns:1fr;margin-top:14px;}
    @media(min-width:960px){.grid{grid-template-columns:1.2fr .8fr;}.span2{grid-column:1/-1;}}
    .card{border:1px solid rgba(255,255,255,.10);background:linear-gradient(180deg, rgba(255,255,255,.08), rgba(255,255,255,.05));
      border-radius:var(--r);box-shadow:var(--shadow);padding:14px;}
    .card h2{margin:0 0 10px;font-size:14px;letter-spacing:.2px;color:rgba(255,255,255,.86);}
    .row{display:flex;gap:10px;flex-wrap:wrap;align-items:center}
    .btn{padding:10px 12px;border-radius:14px;border:1px solid rgba(255,255,255,.16);background:rgba(255,255,255,.06);
      color:var(--text);cursor:pointer;user-select:none;display:inline-flex;gap:8px;align-items:center;
      transition:transform .08s ease, background .2s ease, border-color .2s ease, opacity .2s ease;}
    .btn:hover{background:rgba(255,255,255,.10)} .btn:active{transform:translateY(1px)}
    .btn.primary{border-color:rgba(120,180,255,.45);background:rgba(120,180,255,.14);}
    .btn.danger{border-color:rgba(255,110,110,.45);background:rgba(255,110,110,.12);}
    .btn.warn{border-color:rgba(255,200,90,.45);background:rgba(255,200,90,.12);}
    .btn.ghost{background:transparent;border-color:rgba(255,255,255,.12);}
    .btn.loading{opacity:.65;pointer-events:none;position:relative;} .btn.loading::after{content:" ...";}
    .btn.ok{outline:2px solid rgba(90,255,170,.22)} .btn.err{outline:2px solid rgba(255,90,90,.22)}
    .kv{display:grid;grid-template-columns:180px 1fr 110px;gap:10px;align-items:center;padding:10px 0;border-top:1px solid rgba(255,255,255,.08);}
    .kv:first-of-type{border-top:none;padding-top:0}
    .k{color:var(--muted);font-size:12.5px}
    input[type="range"]{width:100%;accent-color:#8ab4ff;}
    .toggle{display:flex;align-items:center;gap:10px;} .toggle input{transform:scale(1.15)}
    select{background:rgba(0,0,0,.55);color:rgba(255,255,255,.95);border:1px solid rgba(255,255,255,.22);
      border-radius:12px;padding:8px 10px;outline:none;width:100%;}
    select:focus{border-color:rgba(140,190,255,.75);box-shadow:0 0 0 3px rgba(140,190,255,.18);}
    select option{background:#0f1526;color:#ffffff;}
    .hint{color:var(--muted);font-size:12.5px;margin-top:8px;line-height:1.35}
    code{background:rgba(0,0,0,.20);border:1px solid rgba(255,255,255,.12);padding:2px 8px;border-radius:999px;color:rgba(255,255,255,.85);}
    table{width:100%;border-collapse:separate;border-spacing:0;overflow:hidden;border-radius:16px;border:1px solid rgba(255,255,255,.10);background:rgba(0,0,0,.12);}
    th,td{padding:10px 10px;border-bottom:1px solid rgba(255,255,255,.07);vertical-align:top;font-size:12.8px;}
    th{text-align:left;color:rgba(255,255,255,.78);background:rgba(255,255,255,.06);}
    tr:last-child td{border-bottom:none}
    .mini{display:flex;flex-wrap:wrap;gap:8px;align-items:center;}
    .mini label{color:var(--muted);font-size:12px}
    #sched .pinputlike{width:120px;}
    pre{margin:0;padding:12px;border-radius:16px;border:1px solid rgba(255,255,255,.12);background:rgba(0,0,0,.18);
      color:rgba(255,255,255,.86);font-size:12.5px;overflow:auto;white-space:pre-wrap;}
    .chips{display:flex;flex-wrap:wrap;gap:8px}
    .chip{padding:8px 10px;border-radius:999px;border:1px solid rgba(255,255,255,.14);background:rgba(0,0,0,.16);
      color:rgba(255,255,255,.86);cursor:pointer;user-select:none;font-size:12.6px;transition:transform .08s ease, background .2s ease, border-color .2s ease;}
    .chip:hover{background:rgba(255,255,255,.08)} .chip:active{transform:translateY(1px)}
    .preview{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:10px;}
    @media(min-width:960px){.preview{grid-template-columns:1fr 1fr 1fr;}}
    .pbox{border:1px solid rgba(255,255,255,.10);background:rgba(0,0,0,.14);border-radius:16px;padding:10px 12px;}
    .pbox .pt{color:var(--muted);font-size:12px}
    .pbox .pv{margin-top:6px;font-variant-numeric:tabular-nums;font-size:13.5px}
    .pbox .warnline{margin-top:8px;font-size:12px;color:rgba(255,200,90,.95);}
    .dot{display:inline-block;width:9px;height:9px;border-radius:999px;margin-right:8px;vertical-align:middle;background:rgba(255,255,255,.25);}
    .dot.ok{background:rgba(90,255,170,.9)} .dot.warn{background:rgba(255,200,90,.95)} .dot.bad{background:rgba(255,90,90,.95)}
    #toast{position:fixed;right:14px;bottom:14px;padding:10px 12px;border:1px solid rgba(255,255,255,.14);border-radius:14px;
      background:rgba(15,20,35,.92);backdrop-filter:blur(10px);box-shadow:0 16px 44px rgba(0,0,0,.42);display:none;
      color:rgba(255,255,255,.92);font-size:12.8px;max-width:70vw;}
    #toast.ok{border-color:rgba(90,255,170,.35)} #toast.err{border-color:rgba(255,90,90,.35)}
    .pinputlike{width:110px;padding:8px 10px;border-radius:12px;border:1px solid rgba(255,255,255,.14);background:rgba(0,0,0,.22);
      color:rgba(255,255,255,.92);font-variant-numeric:tabular-nums;font-size:12.7px;outline:none;cursor:pointer;text-align:center;}
    .pinputlike:focus{border-color:rgba(140,190,255,.75);box-shadow:0 0 0 3px rgba(140,190,255,.18);}
    .poverlay{position:fixed;inset:0;z-index:9999;display:none;background:rgba(0,0,0,.55);backdrop-filter:blur(8px);}
    .pmodal{position:absolute;left:50%;top:50%;transform:translate(-50%,-50%);width:min(520px, calc(100vw - 24px));
      border:1px solid rgba(255,255,255,.16);border-radius:18px;background:rgba(15,20,35,.96);box-shadow:0 18px 60px rgba(0,0,0,.55);padding:14px;}
    .phead{display:flex;align-items:center;justify-content:space-between;gap:10px;}
    .ptitle{font-size:14px;color:rgba(255,255,255,.88);margin:0;}
    .psub{font-size:12px;color:rgba(255,255,255,.62);margin-top:4px;}
    .pbtn{padding:10px 12px;border-radius:14px;border:1px solid rgba(255,255,255,.16);background:rgba(255,255,255,.06);
      color:rgba(255,255,255,.92);cursor:pointer;}
    .pbtn.primary{border-color:rgba(120,180,255,.45);background:rgba(120,180,255,.14);}
    .pbtn.danger{border-color:rgba(255,110,110,.45);background:rgba(255,110,110,.12);}
    .pbody{margin-top:12px;}
    .prow{display:flex;gap:10px;flex-wrap:wrap;align-items:center;justify-content:space-between;}
    .pval{font-variant-numeric:tabular-nums;font-size:22px;padding:10px 12px;border-radius:14px;border:1px solid rgba(255,255,255,.14);
      background:rgba(0,0,0,.20);min-width:160px;text-align:center;}
    .bigstep{display:flex;gap:10px;align-items:center;}
    .bigstep button{width:56px;height:52px;border-radius:16px;font-size:22px;}
    .pwheel{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:12px;}
    .col{border:1px solid rgba(255,255,255,.12);border-radius:16px;background:rgba(0,0,0,.18);overflow:auto;max-height:240px;-webkit-overflow-scrolling:touch;}
    .col .ci{padding:12px 12px;border-bottom:1px solid rgba(255,255,255,.07);text-align:center;cursor:pointer;user-select:none;font-variant-numeric:tabular-nums;}
    .col .ci:last-child{border-bottom:none;}
    .col .ci:hover{background:rgba(255,255,255,.08)} .col .ci.sel{background:rgba(120,180,255,.18);}
  </style>
</head>
<body>
  <div class="topbar">
    <div class="topbar-inner">
      <div class="title-row">
        <div class="title"><h1 id="devTitle">LaserCat</h1></div>
        <div class="row">
          <span id="pillRun" class="pill"><strong>…</strong></span>
          <span id="pillInfo" class="pill">…</span>
        </div>
      </div>
      <div class="row" style="margin-top:12px">
        <button class="btn primary" onclick="setRun(true, this)">▶ Start</button>
        <button class="btn danger" onclick="setRun(false, this)">■ Stop</button>

        <button class="btn warn" onclick="windowOff(this)">⛔ Fenster aus</button>
        <button class="btn" onclick="windowOn(this)">✅ Fenster an</button>

        <button class="btn" onclick="setPark(this)">📍 Park übernehmen</button>
        <button class="btn" onclick="save(this)">💾 Speichern</button>
        <button class="btn ghost" onclick="reload(this)">↻ Neu laden</button>
      </div>
      <div class="sub" style="margin-top:10px">
        MQTT Base: <code id="base"></code>
      </div>
    </div>
  </div>

  <div class="wrap">
    <div class="grid">
      <div class="card">
        <h2>Presets</h2>
        <div class="chips">
          <div class="chip" onclick="presetMotion('soft')">Sanft</div>
          <div class="chip" onclick="presetMotion('normal')">Normal</div>
          <div class="chip" onclick="presetMotion('wild')">Wild</div>
          <div class="chip" onclick="presetMotion('smart')">Smart Cat</div>
          <div class="chip" onclick="presetArea('small')">Kleiner Bereich</div>
          <div class="chip" onclick="presetArea('large')">Großer Bereich</div>
        </div>
        <div class="hint">Presets ändern nur Werte in der UI. Erst mit <strong>Speichern</strong> wird es übernommen.</div>

        <div class="preview">
          <div class="pbox"><div class="pt">Safe Bounds (mit Edge Margin)</div><div class="pv" id="pvSafe">…</div></div>
          <div class="pbox"><div class="pt">Outputs</div><div class="pv" id="pvOut">…</div></div>
          <div class="pbox"><div class="pt">Netzwerk</div><div class="pv" id="pvNet">…</div></div>
          <div class="pbox"><div class="pt">Zeit</div><div class="pv" id="pvTime">…</div></div>
          <div class="pbox">
            <div class="pt" id="pvSessionTitle">Session Restlaufzeit</div>
            <div class="pv" id="pvSessionRemain">…</div>
            <div class="warnline" id="pvSessionWarn" style="display:none">⚠ Zeit nicht gesetzt (NTP) – keine Uhrzeit möglich</div>
          </div>
          <div class="pbox">
            <div class="pt" id="pvNextTitle">Nächste Bewegung</div>
            <div class="pv" id="pvNextMove">…</div>
            <div class="warnline" id="pvNextWarn" style="display:none">⚠ Zeit nicht gesetzt (NTP) – keine Uhrzeit möglich</div>
          </div>
        </div>
      </div>

      <div class="card">
        <h2>Bewegung</h2>
        <div class="kv"><div class="k">Speed (°/s)</div>
          <input id="speed" type="range" min="5" max="240" step="1" oninput="onRange('speed')">
          <input id="speedN" class="pinputlike" type="text" readonly onclick="pickNum('speed')"></div>
        <div class="kv"><div class="k">Verweilen Min (ms)</div>
          <input id="dwellMin" type="range" min="0" max="2000" step="10" oninput="onRange('dwellMin')">
          <input id="dwellMinN" class="pinputlike" type="text" readonly onclick="pickNum('dwellMin')"></div>
        <div class="kv"><div class="k">Verweilen Max (ms)</div>
          <input id="dwellMax" type="range" min="0" max="4000" step="10" oninput="onRange('dwellMax')">
          <input id="dwellMaxN" class="pinputlike" type="text" readonly onclick="pickNum('dwellMax')"></div>
        <div class="kv"><div class="k">Mode</div>
          <div><select id="mode">
            <option value="0">Random</option><option value="1">Orbit</option><option value="2">Scan</option><option value="3">Smart Cat</option>
          </select></div><div></div></div>
        <div class="kv"><div class="k">Laser</div>
          <div class="toggle"><input id="laserEnabled" type="checkbox"><label for="laserEnabled" style="color:var(--muted)">aktiv</label></div><div></div></div>
        <div class="kv"><div class="k">Session max (min)</div>
          <input id="sessionMaxMin" type="range" min="0" max="60" step="1" oninput="onRange('sessionMaxMin')">
          <input id="sessionMaxMinN" class="pinputlike" type="text" readonly onclick="pickNum('sessionMaxMin')"></div>
        <div class="kv"><div class="k">Cooldown (min)</div>
          <input id="cooldownMin" type="range" min="0" max="60" step="1" oninput="onRange('cooldownMin')">
          <input id="cooldownMinN" class="pinputlike" type="text" readonly onclick="pickNum('cooldownMin')"></div>
        <div class="hint">Tipp: Session=10, Cooldown=2 ergibt im Zeitfenster: 10min an, 2min aus, 10min an, ...</div>
      </div>

      <div class="card span2">
        <h2>Bereich & Anti-Ecken</h2>
        <div class="kv"><div class="k">Pan Min</div>
          <input id="panMin" type="range" min="0" max="180" step="1" oninput="onRange('panMin')">
          <input id="panMinN" class="pinputlike" type="text" readonly onclick="pickNum('panMin')"></div>
        <div class="kv"><div class="k">Pan Max</div>
          <input id="panMax" type="range" min="0" max="180" step="1" oninput="onRange('panMax')">
          <input id="panMaxN" class="pinputlike" type="text" readonly onclick="pickNum('panMax')"></div>
        <div class="kv"><div class="k">Tilt Min</div>
          <input id="tiltMin" type="range" min="0" max="180" step="1" oninput="onRange('tiltMin')">
          <input id="tiltMinN" class="pinputlike" type="text" readonly onclick="pickNum('tiltMin')"></div>
        <div class="kv"><div class="k">Tilt Max</div>
          <input id="tiltMax" type="range" min="0" max="180" step="1" oninput="onRange('tiltMax')">
          <input id="tiltMaxN" class="pinputlike" type="text" readonly onclick="pickNum('tiltMax')"></div>
        <div class="kv"><div class="k">Edge Margin (°)</div>
          <input id="edgeMarginDeg" type="range" min="0" max="30" step="1" oninput="onRange('edgeMarginDeg')">
          <input id="edgeMarginDegN" class="pinputlike" type="text" readonly onclick="pickNum('edgeMarginDeg')"></div>
        <div class="hint">„Edge Margin“ hält Ziele automatisch vom Rand fern (schont SG90, weniger Anschlag).</div>
      </div>

      <div class="card span2">
        <h2>Wochenplan (2 Fenster pro Tag)</h2>
        <table>
          <thead><tr><th style="width:90px">Tag</th><th style="width:110px">Aktiv</th><th>Fenster A</th><th>Fenster B</th></tr></thead>
          <tbody id="sched"></tbody>
        </table>
        <div class="hint">Speichern ist blockiert wenn Stop ≤ Start oder A/B sich überschneiden.</div>
      </div>

      <div class="card"><h2>Status</h2><pre id="state"></pre></div>
    </div>
  </div>

  <div id="toast"></div>

  <div id="poverlay" class="poverlay" onclick="pickerCancel(event)">
    <div class="pmodal" onclick="event.stopPropagation()">
      <div class="phead">
        <div><h3 id="ptitle" class="ptitle">Wert wählen</h3><div id="psub" class="psub"></div></div>
        <div class="row">
          <button class="pbtn danger" onclick="pickerClose(false)">Abbrechen</button>
          <button class="pbtn primary" onclick="pickerClose(true)">OK</button>
        </div>
      </div>
      <div id="pcontent" class="pbody"></div>
    </div>
  </div>

<script>
  const days = ["So","Mo","Di","Mi","Do","Fr","Sa"];
  let cfg = null;

  function toast(msg, kind="ok"){
    const t = document.getElementById("toast");
    t.className = kind; t.textContent = msg; t.style.display = "block";
    clearTimeout(window.__toastTimer);
    window.__toastTimer = setTimeout(()=> t.style.display="none", 1800);
  }

  async function withBtnFeedback(btnEl, fn, okMsg){
    if(btnEl){ btnEl.classList.add("loading"); btnEl.classList.remove("ok","err"); }
    try{
      const res = await fn();
      if(btnEl){ btnEl.classList.add("ok"); setTimeout(()=>btnEl.classList.remove("ok"), 600); }
      toast(okMsg || "OK", "ok"); return res;
    }catch(e){
      if(btnEl){ btnEl.classList.add("err"); setTimeout(()=>btnEl.classList.remove("err"), 900); }
      toast("Fehler", "err"); throw e;
    }finally{ if(btnEl) btnEl.classList.remove("loading"); }
  }

  function minToHHMM(m){
    m = parseInt(m||0,10);
    const hh = String(Math.floor(m/60)).padStart(2,'0');
    const mm = String(m%60).padStart(2,'0');
    return `${hh}:${mm}`;
  }
  function hhmmToMin(s){
    if(!s || s.length<4) return 0;
    const hh = parseInt(s.slice(0,2),10)||0;
    const mm = parseInt(s.slice(3,5),10)||0;
    return Math.max(0, Math.min(23,hh))*60 + Math.max(0,Math.min(59,mm));
  }

  function presetLabel(id){
    id = Number(id||0);
    if(id===1) return "soft";
    if(id===2) return "normal";
    if(id===3) return "wild";
    if(id===4) return "smart";
    return "custom";
  }

  let __picker = { open:false, kind:null, onOk:null, onCancel:null, value:null, meta:null };

  function pickerCancel(){ pickerClose(false); }
  function pickerClose(ok){
    document.getElementById("poverlay").style.display = "none";
    __picker.open = false;
    if(ok && __picker.onOk) __picker.onOk(__picker.value, __picker.meta);
    if(!ok && __picker.onCancel) __picker.onCancel();
    __picker.kind=null; __picker.onOk=null; __picker.onCancel=null; __picker.value=null; __picker.meta=null;
  }
  function pickerShow(title, sub, innerHtml){
    document.getElementById("ptitle").textContent = title;
    document.getElementById("psub").textContent = sub || "";
    document.getElementById("pcontent").innerHTML = innerHtml;
    document.getElementById("poverlay").style.display = "block";
    __picker.open = true;
  }
  function formatNum(v, unit){
    const s = (Math.abs(v - Math.round(v)) < 1e-6) ? String(Math.round(v)) : String(v);
    return unit ? (s + " " + unit) : s;
  }
  function holdRepeat(btn, stepFn){
    let t = null;
    const down = ()=>{
      stepFn();
      let interval = 220;
      t = setInterval(()=>{ stepFn(); interval = Math.max(60, interval - 10); }, interval);
    };
    const up = ()=>{ if(t){ clearInterval(t); t=null; } };
    btn.addEventListener("mousedown", down);
    btn.addEventListener("touchstart", (e)=>{ e.preventDefault(); down(); }, {passive:false});
    window.addEventListener("mouseup", up);
    window.addEventListener("touchend", up);
    window.addEventListener("touchcancel", up);
  }

  function openNumberPicker(opts){
    const mn = Number(opts.min), mx = Number(opts.max), st = Number(opts.step || 1);
    let v = Number(opts.value);
    if(Number.isNaN(v)) v = mn;
    v = Math.max(mn, Math.min(mx, v));
    v = Math.round(v / st) * st;

    __picker.kind = "num";
    __picker.value = v;
    __picker.meta = { min:mn, max:mx, step:st, unit: opts.unit || "" };
    __picker.onOk = opts.onOk;

    const html = `
      <div class="prow">
        <div class="bigstep">
          <button id="pminus" class="pbtn">−</button>
          <div id="pval" class="pval">${formatNum(__picker.value, __picker.meta.unit)}</div>
          <button id="pplus" class="pbtn">+</button>
        </div>
        <div style="flex:1; min-width:220px">
          <input id="pslider" type="range" min="${mn}" max="${mx}" step="${st}" value="${__picker.value}" style="width:100%">
        </div>
      </div>
      <div class="hint" style="margin-top:10px">Tipp: +/− gedrückt halten für schnelleres Verstellen.</div>
    `;
    pickerShow(opts.title || "Wert wählen", opts.sub || "", html);

    const pval = document.getElementById("pval");
    const slider = document.getElementById("pslider");
    const minus = document.getElementById("pminus");
    const plus  = document.getElementById("pplus");

    function setVal(n){
      n = Math.max(mn, Math.min(mx, n));
      n = Math.round(n / st) * st;
      __picker.value = n;
      pval.textContent = formatNum(n, __picker.meta.unit);
      slider.value = String(n);
    }
    slider.addEventListener("input", ()=> setVal(Number(slider.value)));
    holdRepeat(minus, ()=> setVal(__picker.value - st));
    holdRepeat(plus,  ()=> setVal(__picker.value + st));
  }

  function openTimePicker(opts){
    let m = parseInt(opts.valueMin ?? 0, 10);
    if(Number.isNaN(m)) m = 0;
    m = Math.max(0, Math.min(1439, m));
    let hh = Math.floor(m/60);
    let mm = m%60;

    __picker.kind = "time";
    __picker.value = { hh, mm };
    __picker.meta = {};
    __picker.onOk = ()=> opts.onOk(__picker.value.hh*60 + __picker.value.mm);

    const hours = Array.from({length:24}, (_,i)=> String(i).padStart(2,"0"));
    const mins  = Array.from({length:60}, (_,i)=> String(i).padStart(2,"0"));

    const html = `
      <div class="prow"><div class="pval" id="ptime">${String(hh).padStart(2,"0")}:${String(mm).padStart(2,"0")}</div></div>
      <div class="pwheel">
        <div class="col" id="phours">${hours.map((x,i)=> `<div class="ci ${i===hh?'sel':''}" data-h="${i}">${x}</div>`).join("")}</div>
        <div class="col" id="pmins">${mins.map((x,i)=> `<div class="ci ${i===mm?'sel':''}" data-m="${i}">${x}</div>`).join("")}</div>
      </div>
      <div class="hint">Tippe Stunde/Minute. Scrollen ist auch möglich.</div>
    `;
    pickerShow(opts.title || "Zeit wählen", opts.sub || "", html);

    const t = document.getElementById("ptime");
    const hcol = document.getElementById("phours");
    const mcol = document.getElementById("pmins");

    function update(){
      t.textContent = `${String(__picker.value.hh).padStart(2,"0")}:${String(__picker.value.mm).padStart(2,"0")}`;
      [...hcol.querySelectorAll(".ci")].forEach(el=> el.classList.toggle("sel", Number(el.dataset.h)===__picker.value.hh));
      [...mcol.querySelectorAll(".ci")].forEach(el=> el.classList.toggle("sel", Number(el.dataset.m)===__picker.value.mm));
    }
    hcol.addEventListener("click",(e)=>{ const el = e.target.closest(".ci"); if(!el) return; __picker.value.hh = Number(el.dataset.h); update(); });
    mcol.addEventListener("click",(e)=>{ const el = e.target.closest(".ci"); if(!el) return; __picker.value.mm = Number(el.dataset.m); update(); });

    setTimeout(()=>{
      const hs = hcol.querySelector(".ci.sel"); if(hs) hs.scrollIntoView({block:"center"});
      const ms = mcol.querySelector(".ci.sel"); if(ms) ms.scrollIntoView({block:"center"});
    }, 0);
  }

  const numMeta = {
    speed:{min:5,max:240,step:1, unit:"°/s", title:"Speed", sub:"Grad pro Sekunde"},
    dwellMin:{min:0,max:2000,step:10, unit:"ms", title:"Verweilen Min", sub:"Mindest-Verweilen pro Punkt"},
    dwellMax:{min:0,max:4000,step:10, unit:"ms", title:"Verweilen Max", sub:"Maximal-Verweilen pro Punkt"},
    sessionMaxMin:{min:0,max:60,step:1, unit:"min", title:"Session max", sub:"0 = unendlich"},
    cooldownMin:{min:0,max:60,step:1, unit:"min", title:"Cooldown", sub:"Pause zwischen Sessions"},
    panMin:{min:0,max:180,step:1, unit:"°", title:"Pan Min", sub:"linke Grenze"},
    panMax:{min:0,max:180,step:1, unit:"°", title:"Pan Max", sub:"rechte Grenze"},
    tiltMin:{min:0,max:180,step:1, unit:"°", title:"Tilt Min", sub:"untere/obere Grenze"},
    tiltMax:{min:0,max:180,step:1, unit:"°", title:"Tilt Max", sub:"untere/obere Grenze"},
    edgeMarginDeg:{min:0,max:30, step:1, unit:"°", title:"Edge Margin", sub:"Abstand zum Rand"},
  };

  function setRangeAndText(id, value){
    const r = document.getElementById(id);
    const t = document.getElementById(id+"N");
    if(r) r.value = String(value);
    if(t){
      const meta = numMeta[id] || {};
      t.value = formatNum(Number(value), meta.unit || "");
    }
  }

  function onRange(id){
    if(!cfg) return;
    const r = document.getElementById(id);
    if(!r) return;
    const v = Number(r.value);

    if(id === "speed") cfg.speed = v;
    else if(id === "dwellMin") cfg.dwellMin = v;
    else if(id === "dwellMax") cfg.dwellMax = v;
    else if(id === "sessionMaxMin") cfg.sessionMaxMin = v;
    else if(id === "cooldownMin") cfg.cooldownMin = v;
    else if(id === "panMin") cfg.panMin = v;
    else if(id === "panMax") cfg.panMax = v;
    else if(id === "tiltMin") cfg.tiltMin = v;
    else if(id === "tiltMax") cfg.tiltMax = v;
    else if(id === "edgeMarginDeg") cfg.edgeMarginDeg = v;

    setRangeAndText(id, v);
  }

  function pickNum(id){
    if(!cfg) return;
    const meta = numMeta[id];
    if(!meta) return;

    let cur = cfg[id] ?? 0;
    openNumberPicker({
      title: meta.title, sub: meta.sub, value: cur,
      min: meta.min, max: meta.max, step: meta.step, unit: meta.unit,
      onOk: (v)=>{ cfg[id] = v; setRangeAndText(id, v); }
    });
  }

  function presetSelectHtml(id){
    return `
      <select id="${id}" style="width:120px">
        <option value="0">Custom</option>
        <option value="1">Sanft</option>
        <option value="2">Normal</option>
        <option value="3">Wild</option>
        <option value="4">Smart</option>
      </select>`;
  }

  function windowCell(prefix){
    return `
      <div class="mini" style="flex-direction:column; align-items:flex-start; gap:8px">
        <div class="mini" style="gap:10px"><label>Aktiv</label><input type="checkbox" id="${prefix}en"></div>
        <div class="mini" style="gap:10px"><label style="width:48px">Preset</label>${presetSelectHtml(prefix + "pr")}</div>
        <div class="mini" style="gap:10px"><label style="width:48px">Start</label><input class="pinputlike" type="text" readonly id="${prefix}st"></div>
        <div class="mini" style="gap:10px"><label style="width:48px">Stop</label><input class="pinputlike" type="text" readonly id="${prefix}sp"></div>
      </div>`;
  }

  function attachTimePicker(inputEl, title){
    inputEl.onclick = ()=>{
      openTimePicker({ title, valueMin: hhmmToMin(inputEl.value), onOk: (min)=>{ inputEl.value = minToHHMM(min); } });
    };
  }

  function buildScheduleUI(){
    const tb = document.getElementById("sched");
    tb.innerHTML = "";
    cfg.days.forEach((d,i)=>{
      const tr = document.createElement("tr");
      tr.innerHTML = `
        <td><strong>${days[i]}</strong></td>
        <td><input type="checkbox" id="day${i}en"></td>
        <td>${windowCell(`d${i}a`)}</td>
        <td>${windowCell(`d${i}b`)}</td>`;
      tb.appendChild(tr);

      document.getElementById(`day${i}en`).checked = !!d.dayEn;

      document.getElementById(`d${i}aen`).checked = !!d.a.en;
      document.getElementById(`d${i}apr`).value = String(d.a.preset ?? 0);
      const ast = document.getElementById(`d${i}ast`);
      const asp = document.getElementById(`d${i}asp`);
      ast.value = minToHHMM(d.a.start);
      asp.value = minToHHMM(d.a.stop);
      attachTimePicker(ast, `${days[i]} Fenster A Start`);
      attachTimePicker(asp, `${days[i]} Fenster A Stop`);

      document.getElementById(`d${i}ben`).checked = !!d.b.en;
      document.getElementById(`d${i}bpr`).value = String(d.b.preset ?? 0);
      const bst = document.getElementById(`d${i}bst`);
      const bsp = document.getElementById(`d${i}bsp`);
      bst.value = minToHHMM(d.b.start);
      bsp.value = minToHHMM(d.b.stop);
      attachTimePicker(bst, `${days[i]} Fenster B Start`);
      attachTimePicker(bsp, `${days[i]} Fenster B Stop`);
    });
  }

  function applyCfgToUI(){
    setRangeAndText("speed", cfg.speed);
    setRangeAndText("dwellMin", cfg.dwellMin);
    setRangeAndText("dwellMax", cfg.dwellMax);
    setRangeAndText("sessionMaxMin", cfg.sessionMaxMin);
    setRangeAndText("cooldownMin", cfg.cooldownMin);
    setRangeAndText("panMin", cfg.panMin);
    setRangeAndText("panMax", cfg.panMax);
    setRangeAndText("tiltMin", cfg.tiltMin);
    setRangeAndText("tiltMax", cfg.tiltMax);
    setRangeAndText("edgeMarginDeg", cfg.edgeMarginDeg);

    document.getElementById("laserEnabled").checked = !!cfg.laserEnabled;
    document.getElementById("mode").value = String(cfg.mode||0);

    buildScheduleUI();

    document.getElementById("base").textContent = cfg.mqttBase || "(unknown)";
    document.getElementById("devTitle").textContent = cfg.deviceName || "LaserCat";
  }

  function collectUIToCfg(){
    const readRange = (id)=> Number(document.getElementById(id).value);

    cfg.speed = readRange("speed");
    cfg.dwellMin = readRange("dwellMin");
    cfg.dwellMax = readRange("dwellMax");
    cfg.sessionMaxMin = readRange("sessionMaxMin");
    cfg.cooldownMin = readRange("cooldownMin");

    cfg.panMin = readRange("panMin");
    cfg.panMax = readRange("panMax");
    cfg.tiltMin = readRange("tiltMin");
    cfg.tiltMax = readRange("tiltMax");
    cfg.edgeMarginDeg = readRange("edgeMarginDeg");

    cfg.laserEnabled = document.getElementById("laserEnabled").checked;
    cfg.mode = parseInt(document.getElementById("mode").value,10)||0;

    cfg.days = cfg.days.map((d,i)=>({
      dayEn: document.getElementById(`day${i}en`).checked,
      a: {
        en: document.getElementById(`d${i}aen`).checked,
        preset: parseInt(document.getElementById(`d${i}apr`).value,10)||0,
        start: hhmmToMin(document.getElementById(`d${i}ast`).value),
        stop:  hhmmToMin(document.getElementById(`d${i}asp`).value),
      },
      b: {
        en: document.getElementById(`d${i}ben`).checked,
        preset: parseInt(document.getElementById(`d${i}bpr`).value,10)||0,
        start: hhmmToMin(document.getElementById(`d${i}bst`).value),
        stop:  hhmmToMin(document.getElementById(`d${i}bsp`).value),
      }
    }));
  }

  function validateSchedule(){
    if(!cfg || !cfg.days) return true;

    function bad(msg){
      toast(msg, "err");
      return false;
    }

    // Stop must be > Start for enabled windows
    for(let i=0;i<7;i++){
      const d = cfg.days[i];
      if(d.a?.en){
        if(!(d.a.stop > d.a.start)) return bad(`${days[i]} Fenster A: Stop muss nach Start liegen`);
      }
      if(d.b?.en){
        if(!(d.b.stop > d.b.start)) return bad(`${days[i]} Fenster B: Stop muss nach Start liegen`);
      }
      // overlap check if both enabled
      if(d.a?.en && d.b?.en){
        const a0=d.a.start, a1=d.a.stop, b0=d.b.start, b1=d.b.stop;
        const overlap = (a0 < b1) && (b0 < a1);
        if(overlap) return bad(`${days[i]}: Fenster A und B überschneiden sich`);
      }
    }
    return true;
  }

  function presetMotion(kind){
    if(!cfg) return;
    if(kind === "soft"){ cfg.speed = 45; cfg.dwellMin = 350; cfg.dwellMax = 1200; cfg.mode = 0; }
    else if(kind === "normal"){ cfg.speed = 75; cfg.dwellMin = 150; cfg.dwellMax = 900; cfg.mode = 0; }
    else if(kind === "wild"){ cfg.speed = 160; cfg.dwellMin = 40; cfg.dwellMax = 320; cfg.mode = 2; }
    else if(kind === "smart"){ cfg.speed = 95; cfg.dwellMin = 80; cfg.dwellMax = 650; cfg.mode = 3; }
    applyCfgToUI(); toast("Preset gesetzt (bitte Speichern)", "ok");
  }

  function presetArea(kind){
    if(!cfg) return;
    const cPan  = (cfg.panMin + cfg.panMax)/2;
    const cTilt = (cfg.tiltMin + cfg.tiltMax)/2;
    function setAround(center, span){
      const mn = Math.max(0, Math.round(center - span/2));
      const mx = Math.min(180, Math.round(center + span/2));
      return [mn, mx];
    }
    if(kind === "small"){
      let [p1,p2] = setAround(cPan, 60);
      let [t1,t2] = setAround(cTilt, 55);
      cfg.panMin=p1; cfg.panMax=p2; cfg.tiltMin=t1; cfg.tiltMax=t2;
    } else if(kind === "large"){
      let [p1,p2] = setAround(cPan, 120);
      let [t1,t2] = setAround(cTilt, 110);
      cfg.panMin=p1; cfg.panMax=p2; cfg.tiltMin=t1; cfg.tiltMax=t2;
    }
    applyCfgToUI(); toast("Bereich-Preset gesetzt (bitte Speichern)", "ok");
  }

  function dotClass(ok, warn){ if(ok) return "ok"; if(warn) return "warn"; return "bad"; }

  function setPills(s){
    const pillRun = document.getElementById("pillRun");
    const pillInfo = document.getElementById("pillInfo");

    pillRun.className = "pill " + (s.run ? (s.active ? "ok" : "warn") : "bad");
    pillRun.innerHTML = `<strong>${s.run ? "RUN" : "STOP"}</strong>` + (s.active ? " · aktiv" : "");

    let info = [];
    if(!s.timeSet) info.push("time not set");
    if(!s.schedOk) info.push("schedule off");
    if(s.inCooldown) info.push("cooldown");
    if(s.windowBlocked){
      const d = (s.blockedDay>=0 && s.blockedDay<7) ? days[s.blockedDay] : "?";
      info.push(`BLOCK:${d}${s.blockedWin||""}`);
    }

    const ap = s.activePresetName || presetLabel(s.activePreset);
    if(s.activeWindow){
      info.push(`preset:${ap} (${days[s.activeDay]||"?"}${s.activeWindow})`);
    }else{
      info.push(`preset:${ap}`);
    }

    info.push(`laserOut:${s.laserOut ? "ON" : "OFF"}`);
    info.push(`servos:${s.servosAttached ? "ON" : "OFF"}`);

    pillInfo.className = "pill";
    pillInfo.textContent = info.join(" · ");
  }

  function fmtTime(epoch){
    if(!epoch || epoch < 0) return "";
    try{
      const d = new Date(epoch * 1000);
      return d.toLocaleTimeString([], {hour:'2-digit', minute:'2-digit', second:'2-digit'});
    }catch(e){ return ""; }
  }

  function setPreview(s){
    const safe = `Pan ${s.safePanMin}..${s.safePanMax} | Tilt ${s.safeTiltMin}..${s.safeTiltMax}`;
    document.getElementById("pvSafe").textContent = safe;

    const ap = s.activePresetName || presetLabel(s.activePreset);
    const win = s.activeWindow ? (days[s.activeDay] + s.activeWindow) : "—";
    const dPreset = `<span class="dot ${dotClass(true,false)}"></span>preset=${ap} · window=${win}`;

    const dActive = `<span class="dot ${dotClass(s.active, s.run)}"></span>active=${s.active ? "1":"0"} · run=${s.run ? "1":"0"} · schedOk=${s.schedOk ? "1":"0"} · cooldown=${s.inCooldown ? "1":"0"}`;
    const dLaser  = `<span class="dot ${dotClass(s.laserOut, false)}"></span>laserOut=${s.laserOut ? "ON":"OFF"} · laserPerm=${s.laser ? "ON":"OFF"}`;
    const dServo  = `<span class="dot ${dotClass(s.servosAttached, s.active)}"></span>servosAttached=${s.servosAttached ? "1":"0"}`;
    const dBlock  = `<span class="dot ${dotClass(!s.windowBlocked,true)}"></span>windowBlocked=${s.windowBlocked ? "1":"0"}`;
    document.getElementById("pvOut").innerHTML = `${dPreset}<br>${dActive}<br>${dLaser}<br>${dServo}<br>${dBlock}`;

    document.getElementById("pvNet").textContent = `IP ${s.ip || "?"} · RSSI ${s.rssi}`;
    document.getElementById("pvTime").textContent = (s.timeSet ? (s.localTime || "(no localTime)") : "timeSet=0 (warte auf NTP)");

    const showWarn = !s.timeSet;
    document.getElementById("pvSessionWarn").style.display = showWarn ? "block" : "none";
    document.getElementById("pvNextWarn").style.display = showWarn ? "block" : "none";

    if(s.sessionRemainSec === -1){
      document.getElementById("pvSessionRemain").textContent = "∞";
    } else if(!s.active){
      if(!s.run) document.getElementById("pvSessionRemain").textContent = "STOP";
      else if(!s.schedOk) document.getElementById("pvSessionRemain").textContent = "Schedule off";
      else if(s.inCooldown) document.getElementById("pvSessionRemain").textContent = "Cooldown";
      else document.getElementById("pvSessionRemain").textContent = "Warte…";
    } else {
      const until = fmtTime(s.sessionEndsAtEpoch);
      document.getElementById("pvSessionRemain").textContent = (s.sessionRemainSec + " s") + (until ? (" (bis " + until + ")") : "");
    }

    let nextTitle = "Nächste Bewegung";
    let nextText = "—";

    if(!s.run){ nextTitle="Status"; nextText="STOP"; }
    else if(!s.schedOk){ nextTitle="Status"; nextText="Schedule off"; }
    else if(s.inCooldown && !s.active){
      nextTitle="Cooldown: Start in";
      const when = fmtTime(s.nextMoveAtEpoch);
      nextText = (s.nextMoveSec === -1) ? "—" : (s.nextMoveSec + " s") + (when ? (" (um " + when + ")") : "");
    } else if(s.active){
      if(s.nextMoveAtEpoch === -1 && s.nextMoveSec === 0){
        nextTitle="Status"; nextText="Bewegung läuft…";
      } else {
        nextTitle="Nächste Bewegung in";
        const when = fmtTime(s.nextMoveAtEpoch);
        nextText = (s.nextMoveSec === -1) ? "—" : (s.nextMoveSec + " s") + (when ? (" (um " + when + ")") : "");
      }
    } else { nextTitle="Status"; nextText="Warte…"; }

    document.getElementById("pvNextTitle").textContent = nextTitle;
    document.getElementById("pvNextMove").textContent = nextText;
  }

  async function reload(btn){
    return withBtnFeedback(btn, async ()=>{
      const r = await fetch('/api/config');
      cfg = await r.json();
      applyCfgToUI();
    }, "Neu geladen");
  }

  async function save(btn){
    collectUIToCfg();
    if(!validateSchedule()) return;
    return withBtnFeedback(btn, async ()=>{
      const r = await fetch('/api/config', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify(cfg)});
      cfg = await r.json();
      applyCfgToUI();
    }, "Gespeichert");
  }

  async function setRun(on, btn){
    return withBtnFeedback(btn, async ()=>{
      await fetch('/api/run?val='+(on?1:0));
      await pollState();
    }, on ? "Gestartet" : "Gestoppt");
  }

  async function windowOff(btn){
    return withBtnFeedback(btn, async ()=>{
      const r = await fetch('/api/windowOff');
      if(!r.ok){
        const t = await r.text();
        throw new Error(t);
      }
      await pollState();
    }, "Aktuelles Fenster deaktiviert");
  }

  async function windowOn(btn){
    return withBtnFeedback(btn, async ()=>{
      await fetch('/api/windowOn');
      await pollState();
    }, "Fenster wieder aktiv");
  }

  async function setPark(btn){
    return withBtnFeedback(btn, async ()=>{
      await fetch('/api/park?set=1');
      await reload(null);
      await pollState();
    }, "Park gesetzt");
  }

  async function pollState(){
    try{
      const r = await fetch('/api/state');
      const s = await r.json();
      document.getElementById("state").textContent = JSON.stringify(s,null,2);
      setPills(s);
      setPreview(s);
    }catch(e){}
  }

  reload(null).then(pollState);
  setInterval(pollState, 1500);
</script>
</body>
</html>
)HTML";

// ===== Web server =====
void setupWeb() {
  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", INDEX_HTML); });

  server.on("/api/config", HTTP_GET, []() {
    String js = configAsJson();
    StaticJsonDocument<4096> doc;
    deserializeJson(doc, js);
    doc["mqttBase"] = MQTT_BASE;
    doc["deviceName"] = DEVICE_NAME;
    String out;
    serializeJson(doc, out);
    server.send(200, "application/json", out);
  });

  server.on("/api/config", HTTP_POST, []() {
    if (!server.hasArg("plain")) { server.send(400, "text/plain", "missing body"); return; }
    String body = server.arg("plain");

    StaticJsonDocument<4096> doc;
    auto err = deserializeJson(doc, body);
    if (err) { server.send(400, "text/plain", "bad json"); return; }

    cfg.panMin = doc["panMin"] | cfg.panMin;
    cfg.panMax = doc["panMax"] | cfg.panMax;
    cfg.tiltMin = doc["tiltMin"] | cfg.tiltMin;
    cfg.tiltMax = doc["tiltMax"] | cfg.tiltMax;
    cfg.edgeMarginDeg = doc["edgeMarginDeg"] | cfg.edgeMarginDeg;

    cfg.speedDegPerSec = clampf(doc["speed"] | cfg.speedDegPerSec, 5.f, 240.f);
    cfg.dwellMsMin = (uint16_t)clampi(doc["dwellMin"] | cfg.dwellMsMin, 0, 8000);
    cfg.dwellMsMax = (uint16_t)clampi(doc["dwellMax"] | cfg.dwellMsMax, 0, 8000);
    if (cfg.dwellMsMin > cfg.dwellMsMax) std::swap(cfg.dwellMsMin, cfg.dwellMsMax);

    cfg.sessionMaxMin = (uint16_t)clampi(doc["sessionMaxMin"] | cfg.sessionMaxMin, 0, 180);
    cfg.cooldownMin = (uint16_t)clampi(doc["cooldownMin"] | cfg.cooldownMin, 0, 180);

    cfg.laserEnabled = doc["laserEnabled"] | cfg.laserEnabled;
    cfg.runEnabled = doc["runEnabled"] | cfg.runEnabled;

    cfg.mode = (MotionMode)clampi(doc["mode"] | (int)cfg.mode, 0, 3);

    JsonArray days = doc["days"].as<JsonArray>();
    if (days && days.size() == 7) {
      for (int i = 0; i < 7; i++) {
        cfg.days[i].dayEnabled = days[i]["dayEn"] | cfg.days[i].dayEnabled;

        JsonObject a = days[i]["a"];
        JsonObject b = days[i]["b"];
        if (a) {
          cfg.days[i].a.enabled = a["en"] | cfg.days[i].a.enabled;
          cfg.days[i].a.startMin = (uint16_t)clampi(a["start"] | cfg.days[i].a.startMin, 0, 24 * 60 - 1);
          cfg.days[i].a.stopMin  = (uint16_t)clampi(a["stop"]  | cfg.days[i].a.stopMin,  0, 24 * 60);
          cfg.days[i].a.preset   = (uint8_t)clampi(a["preset"] | cfg.days[i].a.preset, 0, 4);
        }
        if (b) {
          cfg.days[i].b.enabled = b["en"] | cfg.days[i].b.enabled;
          cfg.days[i].b.startMin = (uint16_t)clampi(b["start"] | cfg.days[i].b.startMin, 0, 24 * 60 - 1);
          cfg.days[i].b.stopMin  = (uint16_t)clampi(b["stop"]  | cfg.days[i].b.stopMin,  0, 24 * 60);
          cfg.days[i].b.preset   = (uint8_t)clampi(b["preset"] | cfg.days[i].b.preset, 0, 4);
        }
      }
    }

    normalizeBounds();
    cycleReset();
    saveConfig();
    mqttPublishState(true);

    String js2 = configAsJson();
    StaticJsonDocument<4096> doc2;
    deserializeJson(doc2, js2);
    doc2["mqttBase"] = MQTT_BASE;
    doc2["deviceName"] = DEVICE_NAME;
    String out;
    serializeJson(doc2, out);
    server.send(200, "application/json", out);
  });

  server.on("/api/run", HTTP_GET, []() {
    String v = server.arg("val");
    cfg.runEnabled = (v == "1" || v == "true" || v == "ON");
    cycleReset();
    saveConfig();
    mqttPublishState(true);
    server.send(200, "text/plain", cfg.runEnabled ? "ON" : "OFF");
  });

  // ---- NEW: Window OFF / ON ----
  server.on("/api/windowOff", HTTP_GET, []() {
    int d; char w; uint8_t p;
    bool ok = getActiveWindowNow(d, w, p);
    if (!ok) { server.send(409, "text/plain", "no active window"); return; }

    windowBlocked = true;
    blockedDay = d;
    blockedWin = w;

    cycleReset(); // stop now
    mqttPublishState(true);
    server.send(200, "text/plain", "OK");
  });

  server.on("/api/windowOn", HTTP_GET, []() {
    windowBlocked = false;
    blockedDay = -1;
    blockedWin = 0;

    cycleReset();
    mqttPublishState(true);
    server.send(200, "text/plain", "OK");
  });

  server.on("/api/park", HTTP_GET, []() {
    String s = server.arg("set");
    if (s == "1") {
      cfg.parkPan  = clampi((int)round(curPan),  0, 180);
      cfg.parkTilt = clampi((int)round(curTilt), 0, 180);
      saveConfig();
      mqttPublishState(true);
    }
    server.send(200, "application/json", configAsJson());
  });

  server.on("/api/state", HTTP_GET, []() {
    StaticJsonDocument<3000> doc;

    // runtime housekeeping
    clearBlockIfExpired();

    // Update active preset info without side effects
    updatePresetOverride(false);

    const bool schedOk = allowedByScheduleNow();
    cycleTick(schedOk);

    const bool active = isActiveNow();
    const bool laserOut = active && cfg.laserEnabled;

    doc["run"] = cfg.runEnabled;
    doc["laser"] = cfg.laserEnabled;
    doc["laserOut"] = laserOut;
    doc["active"] = active;

    doc["mode"] = (int)cfg.mode;
    doc["pan"] = (int)curPan;
    doc["tilt"] = (int)curTilt;
    doc["parkPan"] = cfg.parkPan;
    doc["parkTilt"] = cfg.parkTilt;
    doc["edgeMarginDeg"] = cfg.edgeMarginDeg;
    doc["safePanMin"] = safePanMin();
    doc["safePanMax"] = safePanMax();
    doc["safeTiltMin"] = safeTiltMin();
    doc["safeTiltMax"] = safeTiltMax();

    doc["timeSet"] = timeIsSet();
    doc["localTime"] = localTimeString();

    doc["schedOk"] = schedOk;
    doc["inCooldown"] = cycleInCooldown();
    doc["servosAttached"] = servosAttached;

    doc["ip"] = WiFi.localIP().toString();
    doc["rssi"] = WiFi.RSSI();

    time_t nowEpoch = time(nullptr);
    doc["epoch"] = (uint32_t)nowEpoch;

    doc["activeDay"] = (int)activeDay;
    doc["activeWindow"] = (activeWin ? String(activeWin) : String(""));
    doc["activePreset"] = (int)activePreset;
    doc["activePresetName"] = presetName(activePreset);

    doc["windowBlocked"] = windowBlocked;
    doc["blockedDay"] = (int)blockedDay;
    doc["blockedWin"] = (blockedWin ? String(blockedWin) : String(""));

    int nextMoveSec = -1;
    int nextMoveAtEpoch = -1;

    if (active) {
      if (dwellUntilMs && msNow() < dwellUntilMs) {
        nextMoveSec = (int)((dwellUntilMs - msNow()) / 1000UL);
        if (nextMoveSec < 0) nextMoveSec = 0;
        if (timeIsSet()) nextMoveAtEpoch = (int)(nowEpoch + nextMoveSec);
      } else {
        nextMoveSec = 0; nextMoveAtEpoch = -1;
      }
    } else {
      if (cfg.runEnabled && schedOk && cycleInCooldown() && cycleCooldownUntilMs > msNow()) {
        nextMoveSec = (int)((cycleCooldownUntilMs - msNow()) / 1000UL);
        if (nextMoveSec < 0) nextMoveSec = 0;
        if (timeIsSet()) nextMoveAtEpoch = (int)(nowEpoch + nextMoveSec);
      }
    }
    doc["nextMoveSec"] = nextMoveSec;
    doc["nextMoveAtEpoch"] = nextMoveAtEpoch;

    int sessionRemainSec = -1;
    int sessionEndsAtEpoch = -1;

    if (cfg.sessionMaxMin == 0) {
      sessionRemainSec = -1; sessionEndsAtEpoch = -1;
    } else if (active && cycleSessionStartMs != 0) {
      uint32_t maxMs = (uint32_t)cfg.sessionMaxMin * 60UL * 1000UL;
      uint32_t endMs = cycleSessionStartMs + maxMs;
      sessionRemainSec = (endMs <= msNow()) ? 0 : (int)((endMs - msNow()) / 1000UL);
      if (timeIsSet()) sessionEndsAtEpoch = (int)(nowEpoch + sessionRemainSec);
    } else {
      sessionRemainSec = 0; sessionEndsAtEpoch = -1;
    }

    doc["sessionRemainSec"] = sessionRemainSec;
    doc["sessionEndsAtEpoch"] = sessionEndsAtEpoch;

    String out;
    serializeJson(doc, out);
    server.send(200, "application/json", out);
  });

  server.begin();
}

// ===== OTA =====
void setupOTA() {
  ArduinoOTA.setHostname(DEVICE_NAME);
  if (OTA_PASSWORD && strlen(OTA_PASSWORD) > 0) ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.begin();
}

// ===== Setup/Loop =====
void setup() {
  Serial.begin(74880);
  Serial.println();
  Serial.println("LaserCat booting...");

  pinMode(PIN_LASER, OUTPUT);
  laserOn(false);

  LittleFS.begin();

  setDefaultSchedule();
  loadConfig();
  normalizeBounds();

  detachServos();
  cycleReset();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t t0 = msNow();
  while (WiFi.status() != WL_CONNECTED && msNow() - t0 < 15000) delay(200);

  Serial.print("WiFi: "); Serial.println(WiFi.status() == WL_CONNECTED ? "OK" : "NOT CONNECTED");
  Serial.print("IP: "); Serial.println(WiFi.localIP());

  setupTime();

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);

  setupWeb();
  setupOTA();

  randomSeed(ESP.getChipId() ^ micros());

  curPan = clampi(cfg.parkPan, 0, 180);
  curTilt = clampi(cfg.parkTilt, 0, 180);
  segStartPan = segTargetPan = curPan;
  segStartTilt = segTargetTilt = curTilt;
  segStartMs = msNow();

  updatePresetOverride(false);
  mqttPublishState(true);
}

void loop() {
  server.handleClient();
  ArduinoOTA.handle();

  if (WiFi.status() == WL_CONNECTED) {
    mqttEnsure();
    mqtt.loop();
  }

  clearBlockIfExpired();
  updatePresetOverride(true);

  const bool active = isActiveNow();

  if (!active) {
    if (wasActiveLastLoop) parkSmoothThenDetach(900);
    else { laserOn(false); detachServos(); }
    wasActiveLastLoop = false;

    static uint32_t lastPub = 0;
    if (msNow() - lastPub > 5000) { mqttPublishState(true); lastPub = msNow(); }
    delay(10);
    return;
  }

  wasActiveLastLoop = true;
  ensureServosAttached();

  laserOn(cfg.laserEnabled);

  if (dwellUntilMs && msNow() < dwellUntilMs) { delay(5); return; }

  uint32_t elapsed = msNow() - segStartMs;
  float t = (segDurMs == 0) ? 1.0f : (float)elapsed / (float)segDurMs;

  if (t >= 1.0f) {
    curPan = segTargetPan;
    curTilt = segTargetTilt;
    writeServos(curPan, curTilt);
    onArrived();
    chooseNextTarget();
    mqttPublishState(false);
    delay(5);
    return;
  }

  float e = smoothstep(t);
  curPan = segStartPan + (segTargetPan - segStartPan) * e;
  curTilt = segStartTilt + (segTargetTilt - segStartTilt) * e;

  int pMin = safePanMin(), pMax = safePanMax();
  int tMin = safeTiltMin(), tMax = safeTiltMax();

  if (pMin < pMax) curPan = clampf(curPan, (float)pMin, (float)pMax);
  else curPan = clampf(curPan, (float)cfg.panMin, (float)cfg.panMax);

  if (tMin < tMax) curTilt = clampf(curTilt, (float)tMin, (float)tMax);
  else curTilt = clampf(curTilt, (float)cfg.tiltMin, (float)cfg.tiltMax);

  writeServos(curPan, curTilt);
  delay(5);
}
