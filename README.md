# Laserpunktspiel-für-Katzen
Bewegter Laserpunkt für Katzenbeschäftigung

# LaserCat (ESP8266 / Wemos D1 mini)
LaserCat ist ein ESP8266-Sketch (Arduino IDE) zur Steuerung eines 2-Achsen Servo-Turrets (Pan/Tilt) mit Laser. Das Projekt bringt eine moderne **Web-UI**, **Wochenplan (2 Fenster/Tag)**, **Preset-Zuordnung pro Zeitfenster**, **Session/Cooldown-Zyklen**, **MQTT** und **OTA** mit.

> ⚠️ Sicherheit: Laser nur verantwortungsvoll einsetzen (nicht in Augen / reflektierende Flächen). Für Haustiere nur unter Aufsicht.

---

## Features

### Web-UI (on-device)
- Live-Status: RUN/STOP, aktiv/inaktiv, Laser-Out, Servo-Status, RSSI/IP, Zeit/NTP
- Komfortable **Number-Picker** (Speed, Dwell, Bounds, Edge Margin, Session/Cooldown)
- **Time-Picker** für Start/Stop je Zeitfenster
- Preset-Buttons: *Sanft / Normal / Wild / Smart Cat* + Bereich *klein/groß*
- Validierung vor dem Speichern:
  - **Stop darf nicht ≤ Start sein**
  - **Fenster A und B dürfen sich nicht überschneiden**

### Wochenplan: 2 Zeitfenster pro Tag (A/B)
- Pro Tag aktivierbar/deaktivierbar
- Fenster A/B separat aktivierbar
- Zeitfenster steuern, wann das Gerät „active“ sein darf (NTP erforderlich)

### Presets pro Zeitfenster
Pro Tag & Zeitfenster (A/B) kann ein Preset zugeordnet werden:
- **Custom** (nutzt UI/Config-Werte)
- **Sanft**
- **Normal**
- **Wild**
- **Smart**

Während ein Fenster aktiv ist, überschreibt das Preset runtime-seitig Speed/Dwell/Mode (Custom = keine Überschreibung).

### „Fenster aus / Fenster an“ (Runtime-Block)
In der UI:
- **Fenster aus** blockiert das aktuell aktive Zeitfenster **temporär** (runtime only, nicht persistent)
- Block wird aufgehoben, sobald ein anderes Fenster aktiv wird oder nach Reboot

### Session/Cooldown-Zyklus
Optionaler Zyklus innerhalb eines aktiven Zeitfensters:
- `sessionMaxMin = 0` → unendlich aktiv
- sonst: `Session` Minuten aktiv → `Cooldown` Minuten Pause → wieder aktiv → …

### Bewegungsmodi
- `Random` – zufällige Zielpunkte im Bereich
- `Orbit` – um die Mitte „kreisen“
- `Scan` – horizontale/vertikale Scans
- `Smart Cat` – dynamische Muster (Bursts, Steps, Dwell-Overrides) für unvorhersehbare Bewegung

### Anti-Ecken / Edge Margin
- „Edge Margin“ hält Ziele automatisch vom Rand fern (schont Servos, weniger Anschlag)
- Safe-Bounds werden in der UI angezeigt

### MQTT
- Publish u.a. `state/run`, `state/laser`
- Subscribe `cmd/run` (ON/OFF)
- Optional: Schedule-Änderungen via MQTT (times/enables)

### OTA
- Hostname = `DEVICE_NAME`
- Passwort = `OTA_PASSWORD` (leer = kein Passwort)

---

## Hardware

### Empfohlene Komponenten
- Wemos D1 mini / ESP8266
- 2× Servo (z.B. SG90/MG90S)
- Laser-Modul (5V)
- MOSFET/Transistor + Widerstände (für Laser-Schalten empfohlen)
- **5V Netzteil ≥ 2A** (für Servos)

### Wichtige Hinweise
- ✅ **Servos mit separater 5V Versorgung** betreiben (≥ 2A empfohlen)
- ✅ **GND gemeinsam**: Servo-Netzteil GND ↔ Wemos GND
- ✅ Laser **nicht direkt** vom ESP-Pin speisen → über **Transistor/MOSFET** an `PIN_LASER` schalten
- ⚠️ Boot-Strap Pins vermeiden (bereits berücksichtigt): **D3/D4/D8**

### Default Pins
- `PIN_SERVO_PAN  = D5 (GPIO14)`
- `PIN_SERVO_TILT = D6 (GPIO12)`
- `PIN_LASER      = D7 (GPIO13)` *(MOSFET empfohlen)*

---

## Quickstart

### 1) Arduino IDE Setup
- Board: **LOLIN(WEMOS) D1 mini** (ESP8266)
- ESP8266 Board Package installiert
- Libraries:
  - ESP8266WiFi (Board package)
  - ESP8266WebServer (Board package)
  - LittleFS (Board package)
  - PubSubClient
  - ArduinoJson **7.x**
  - Servo
  - ArduinoOTA

### 2) Konfiguration im Sketch
Passe die Konstanten an:
- `DEVICE_NAME` (auch MQTT Base + UI Titel)
- `OTA_PASSWORD`
- `WIFI_SSID` / `WIFI_PASS`
- `MQTT_HOST` / `MQTT_PORT` / `MQTT_USER` / `MQTT_PASS`

Optional:
- `MQTT_BASE = String(DEVICE_NAME);` (Base Topic)

### 3) Flashen
- USB verbinden, richtigen Port wählen
- Sketch hochladen

Beim ersten Start:
- Default Schedule wird gesetzt
- Config wird aus `/config.json` geladen, falls vorhanden
- Webserver startet auf Port 80

### 4) Web-UI öffnen
Im Serial Monitor wird die IP ausgegeben (oder am Router nachsehen).  
Dann im Browser öffnen:
- `http://<IP-des-ESP>/`

---

## Konfiguration & Persistenz

- Config wird als JSON in **LittleFS** gespeichert:
  - Pfad: `/config.json`
- Web-UI lädt/speichert über `/api/config`

---

## REST API

### `GET /api/config`
Gibt die aktuelle Konfiguration zurück (inkl. `mqttBase` und `deviceName`).

### `POST /api/config`
Speichert Konfiguration (JSON Body). Rückgabe: gespeicherte Config.

> Tipp: In der Web-UI wird zusätzlich validiert:
> - Stop > Start
> - A/B überschneiden sich nicht

### `GET /api/state`
Live-Status als JSON, u.a.:
- `run`, `active`, `schedOk`, `inCooldown`
- `laser`, `laserOut`, `servosAttached`
- `pan`, `tilt`, `safePanMin/Max`, `safeTiltMin/Max`
- NTP/Time: `timeSet`, `localTime`, `epoch`
- Preset: `activeDay`, `activeWindow`, `activePreset`, `activePresetName`
- Window-Block: `windowBlocked`, `blockedDay`, `blockedWin`
- Next: `nextMoveSec`, `nextMoveAtEpoch`
- Session: `sessionRemainSec`, `sessionEndsAtEpoch`

### `GET /api/run?val=1|0`
Start/Stop (setzt `runEnabled` persistent + MQTT publish).

### `GET /api/windowOff`
Blockiert das **aktuell aktive Zeitfenster** runtime-seitig.  
Antwort:
- `200 OK` wenn blockiert
- `409` wenn kein Fenster aktiv ist

### `GET /api/windowOn`
Hebt die Blockierung wieder auf.

### `GET /api/park?set=1`
Setzt aktuelle Servo-Position als Parkposition (persistent).

---

## MQTT

### Base Topic
Standard: `MQTT_BASE = DEVICE_NAME`

### Publish (retained)
- `<base>/state/run` → `ON|OFF`
- `<base>/state/laser` → `ON|OFF` *(Laser-Out abhängig von active & laserEnabled)*

### Subscribe
- `<base>/cmd/run` → `ON|OFF|1|0|true|false`

### Optional: Schedule Commands (times/enables)
Themenstruktur (Beispiele):
- `<base>/cmd/day/<day>/dayEn` → `ON|OFF`
- `<base>/cmd/day/<day>/a/en` → `ON|OFF`
- `<base>/cmd/day/<day>/a/start` → Minuten 0..1439
- `<base>/cmd/day/<day>/a/stop` → Minuten 0..1440
- `<base>/cmd/day/<day>/b/en` → `ON|OFF`
- `<base>/cmd/day/<day>/b/start` → Minuten 0..1439
- `<base>/cmd/day/<day>/b/stop` → Minuten 0..1440

`<day>` ist 0..6 (So..Sa) entsprechend `tm_wday`.

---

## Presets

Preset-IDs:
- `0` = Custom
- `1` = Sanft
- `2` = Normal
- `3` = Wild
- `4` = Smart

Preset-Defaults (runtime override):
- **Sanft**: Speed 45°/s, Dwell 350..1200ms, Mode Random  
- **Normal**: Speed 75°/s, Dwell 150..900ms, Mode Random  
- **Wild**: Speed 160°/s, Dwell 40..320ms, Mode Scan  
- **Smart**: Speed 95°/s, Dwell 80..650ms, Mode Smart Cat

---

## Troubleshooting

### Zeit/NTP wird nicht gesetzt
- Ohne `timeSet=1` ist Scheduling deaktiviert (`allowedByScheduleNow()` → false)
- Prüfe WLAN/Internet-Zugang (NTP braucht Verbindung)
- Firewall/Router: UDP Port 123 (NTP)

### Servos zucken / Reset / instabil
- Separate 5V Versorgung nutzen (≥ 2A)
- GND gemeinsam verbinden
- Kabel kurz halten
- Optional: Kondensator (z.B. 470–1000µF) an 5V Servo-Schiene

### Laser flackert / ESP wird warm
- Laser nicht direkt am ESP-Pin betreiben
- MOSFET/Transistor verwenden, ggf. Gate/Base Widerstand

### OTA klappt nicht
- Gerät muss im gleichen Netz erreichbar sein
- Hostname = `DEVICE_NAME`
- Passwort korrekt (`OTA_PASSWORD`)
- Port/Firewall prüfen

---

## Lizenz
Wähle eine Lizenz (z.B. MIT) und füge eine `LICENSE` Datei hinzu.
