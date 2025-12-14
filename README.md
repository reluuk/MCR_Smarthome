# ESP32 Umwelt-Datenlogger – Projektdokumentation

**Projekt:** SmartHome im Fach Microcontroller – Batteriebetriebener IoT-Datenlogger  
**Entwickler:** Lukas Reif in Kollaboration mit Niklas Bräu  
**Datum:** November 2025  
**Plattform/MCU:** ESP32 DevKit V1 (az-delivery-devkit-v4)  

**Hinweis**: Dieses Projekt wurde im Rahmen der Techniker-Ausbildung (MCR) entwickelt. Der Code steht unter MIT-Lizenz und darf frei verwendet werden. Für produktiven Einsatz wird empfohlen, die MQTT-Credentials in einer separaten, nicht versionierten Headerdatei zu speichern.

---

## 1. Projektübersicht

### 1.1 Zielsetzung

Entwicklung eines energieeffizienten, autonomen Datenloggers zur kontinuierlichen Erfassung von Umweltdaten (Temperatur, Luftfeuchtigkeit, Luftdruck) mit drahtloser Übertragung in die Cloud und lokaler Speicherung auf SD-Karte.

### 1.2 Kernmerkmale

- **Energieeffizient:** Deep-Sleep-Modus mit ~10 µA Stromaufnahme zwischen Messungen
- **Robust:** State-Machine-Architektur mit automatischer Fehlerbehandlung
- **Konfigurierbar:** Web-Interface zur Laufzeitkonfiguration von WiFi und MQTT
- **Sicher:** TLS-verschlüsselte MQTT-Verbindung zu EMQX Cloud
- **Lokal redundant:** CSV-Speicherung auf SD-Karte mit ISO 8601 Zeitstempeln
- **Benutzerfreundlich:** LED-Feedback und Tasterbedienung für Status und Konfiguration

### 1.3 Technologie-Stack

| Kategorie           | Technologie                                  |
|---------------------|----------------------------------------------|
| Mikrocontroller     | ESP32 (Dual-Core, 240 MHz, WiFi/BT)          |
| Framework           | Arduino (PlatformIO)                         |
| Programmiersprache  | C++                                          |
| Protokolle          | MQTT over TLS, HTTP, NTP, I2C, SPI           |
| Cloud-Plattform     | EMQX Cloud (Managed MQTT Broker)             |
| Zeitsynchronisation | NTP mit CET/CEST (Sommerzeit)                |
| Persistenz          | NVS (Non-Volatile Storage), SD-Karte (FAT32) |

---

## 2. Hardware-Komponenten

### 2.1 Übersicht
```
  ┌───────────────────────────────────────────────────────────────────┐
  │ ESP32 DevKit V1                                                   │
  │  ├─ GPIO14 ─── Taster ─────────────── GND  (Config-Mode Toggle)   │
  │  ├─ GPIO17 ─── LED Grün ──── 220Ω ─┬─ GND  (WiFi/MQTT Status)     │
  │  ├─ GPIO16 ─── LED Rot  ──── 220Ω  ┘       (Error Status)         │
  │  ├─ GPIO21 ─── I2C SDA ────────────── BME280 (0x76)               │
  │  ├─ GPIO22 ─── I2C SCL ───────────────┘                           │
  │  ├─ GPIO5  ─── SD CS ──────────────── SD-Karten Modul             │
  │  ├─ GPIO23 ─── SPI MOSI ──────────────┘││                         │
  │  ├─ GPIO19 ─── SPI MISO ───────────────┘│                         │
  │  └─ GPIO18 ─── SPI SCK ─────────────────┘                         │
  └───────────────────────────────────────────────────────────────────┘
```

### 2.2 Komponentenliste

| Komponente      | Modell/Typ          | Funktion                  | Interface        |
|-----------------|---------------------|---------------------------|------------------|
| Mikrocontroller | ESP32 DevKit V1     | Hauptsteuerung            | -                |
| Umweltsensor    | BME280              | Temp., Luftfeuchte, Druck | I2C (0x76)       |
| Speicher        | MicroSD Modul       | Lokale Datenarchivierung  | SPI (CS=GPIO5)   |
| Status-LED      | 5mm LED grün        | WiFi/MQTT Status          | GPIO17 (PWM)     |
| Fehler-LED      | 5mm LED rot         | Fehleranzeige             | GPIO16 (digital) |
| Taster          | Push Button         | Konfigurationsmodus       | GPIO14 (Pull-Up) |
| Stromversorgung | 18650 Li-Ion (3.7V) | Mobile Energieversorgung  | 5V Step-Up       |

### 2.3 Energieversorgung

**Konzept:** 18650 Li-Ion Akku (2600-3000 mAh) mit 5V Step-Up Converter

**Stromverbrauch:**
- Deep-Sleep: ~10 µA
- WiFi aktiv: ~160 mA
- Messzyklus (60s): ~1s aktiv → 0.27 mAh/h

**Laufzeitberechnung (2600 mAh Akku, 60s Intervall):**
Energiebedarf pro Stunde = (1s × 160mA + 3599s × 0.01mA) / 3600s
= 0.27 mAh/h

Laufzeit = 2600 mAh / 0.27 mAh/h ≈ 9630 h ≈ 401 Tage


---

## 3. Software-Architektur

### 3.1 State Machine

**Das System basiert auf einer State Machine mit 7 Zuständen:**
```
                           ┌───────────────┐
         ┌─────────────────│   STATE_INIT  │──────────────┐
         │                 └───────────────┘              │
         │ WiFi/MQTT i.O.                                 │ Taster wurde gedrückt
         │ SD/Sensor i.O.                                 │
         ▼                                                ▼
┌──────────────────┐                             ┌──────────────────┐
│                  │ Taster wurde gedrückt       │                  │
│                  │────────────────────────────►│                  │
│   STATE_NORMAL   │◄────────────────────────────│   STATE_CONFIG   │
│                  │ Taster wurde gedrückt/      │                  │
│                  │ Reset ausgelöst             │                  │
└────┬─────┬───────┘                             └──────────────────┘
     │     │
Error│     │Timeout
     │     │
┌────▼─────▼───────────────────────────────┐
│ STATE_ERROR_WIFI / _MQTT / _OTHER        │
└────┬──────────────────────────────┬──────┘
     │                              │
     │ Recovery                     │ 10min Timeout
     │                              │
     └──────────────────────────────┼─────────┐
                                    ▼         │
                          ┌──────────────────┐│
                          │ STATE_DEEP_SLEEP ││
                          │     _ONLY        ││
                          └──────────────────┘│
                                    │         │
                                    └─────────┘
```
**Zustandsbeschreibungen:**

| State             | Beschreibung                    | LED Grün       | LED Rot        | Timeout    |
|-------------------|---------------------------------|----------------|----------------|------------|
| `INIT`            | Systeminitialisierung           | Blinkt langsam | Aus            | -          |
| `NORMAL`          | Normalbetrieb, Messungen aktiv  | An             | Aus            | -          |
| `CONFIG_MANUAL`   | Manueller Konfigurationsmodus   | An             | Blinkt langsam | 5 min      |
| `ERROR_WIFI`      | WiFi-Verbindungsfehler          | Aus            | Blinkt schnell | Auto-Retry |
| `ERROR_MQTT`      | MQTT-Verbindungsfehler          | Blinkt schnell | An             | Auto-Retry |
| `ERROR_OTHER`     | Sensor-/SD-Fehler               | Aus            | An             | 10 min     |
| `DEEP_SLEEP_ONLY` | Notfall-Sleep (nur Reset weckt) | Aus            | Aus            | ∞          |

### 3.2 Programmstruktur

```
// Hauptmodule
├── Hardware-Initialisierung (GPIO, I2C, SPI, PWM)
├── NVS-Konfigurationsverwaltung
├── WiFi-Management (STA + AP Modi)
├── MQTT-Client (TLS-gesichert)
├── Web-Server (Konfigurationsinterface)
├── LED-Steuerung (PWM-Dimming)
├── Button-Handler (Debouncing)
├── BME280-Sensorabfrage
├── SD-Karten-Logger (CSV)
├── NTP-Zeitsynchronisation (CET/CEST)
├── Deep-Sleep-Management
└── State Machine (Loop-Dispatcher)
```

### 3.2 Datenfluss
```
[BME280 Sensor] ──I2C──► [ESP32] ──MQTT/TLS──► [EMQX Cloud]
                            │                        
                            └─────SPI────► [SD-Karte]
                                               │
                                               ▼
                                     [CSV: smarthome.csv]
```

---
## 4. Verwendete Bibliotheken 

### 4.1 Core Libraries (Arduino ESP32)

|Bibliothek |Zweck|
|-----------------|---------------------------|
| WiFi.h| WiFi-Verbindung (STA/AP Modus) |
| WiFiClientSecure.h| TLS/SSL-Client für MQTT |
| WebServer.h| HTTP-Server für Web-Interface |
| Preferences.h| NVS-Speicher (Flash-Persistenz) |
| Wire.h| I2C-Kommunikation (BME280)|
| SPI.h| SPI-Bus (SD-Karte) |
| SD.h| SD-Karten-Dateisystem |
| time.h|	Zeitfunktionen (NTP, Timezone) |

### 4.2 Externe Bibliotheken (Platform IO)
```ini
lib_deps =
    knolleary/PubSubClient @ ^2.8
    bblanchon/ArduinoJson @ ^6
    adafruit/Adafruit BME280 Library @ ^2.2.4
    adafruit/Adafruit Unified Sensor @ ^1.1.14
```
|Bibliothek |Version |Lizenz |Zweck |
|------------|-------------|-----------------|-----------------|
| PubSubClient |	2.8 |	MIT	| MQTT-Client (TLS-Support) |
| ArduinoJson |	6.x |	MIT |	JSON-Serialisierung |
| Adafruit_BME280 |	2.2.4 |	BSD |	BME280-Sensor-Treiber|
| Adafruit_Unified_Sensor |	1.1.14 |	Apache |	Sensor-Abstraktionsschicht |

---
## 5. Komponenten Funktionsweise

### 5.1 WiFi Provisioning

**Funktionsprinzip:**
  1. Boot: System prüft gespeicherte Credentials in NVS
  2. Keine Credentials: Startet Access Point [ESP32-SSID]
  3. Mit Credentials: Verbindungsversuch (15s Timeout)
  4. Fehler: Fallback zu AP-Modus
  5. Manuell: Taster-Druck wechselt zu Config-Modus

**AP-Konfiguration:**
  - SSID: [AP-SSID]
  - Passwort: [AP-Passwort]
  
**Code-Beispiel (verkürzt):**
```cpp
bool connectWiFi(uint32_t timeoutMs = 15000) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());
  
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - start > timeoutMs) return false;
    delay(100);
  }
  return true;
}
```

### 5.2 MQTT over TLS

**Broker-Konfiguration:**
  - Host: [Adresse z.B. mqtt.com]
  - Port: [Portnummer z.B. 8883 (MQTT over TLS)]
  - Protokoll: MQTTv3.1.1
  - QoS: 0 (Best Effort)
  - Retained: Ja (Status + State Topics)

**Topics:**
- smarthome/senderlukas/status   → "online"/"offline" (retained)
- smarthome/senderlukas/state    → JSON System-Status (retained)
- smarthome/senderlukas/env      → JSON Messdaten

**TLS-Zertifikat:**

- DigiCert Global Root CA (eingebettet in secrets.h)

**Last Will Testament:**
```cpp
mqtt.setServer(mqttHost.c_str(), mqttPort);
mqtt.connect(clientId, mqttUser, mqttPass, "smarthome/senderniklas/status", 0, true, "offline");
```

**JSON-Payload (env):**
```json
{
  "ts": 1732972425,
  "temp": 23.45,
  "hum": 65.32,
  "pres": 1013.25,
}
```
### 5.3 BME280 Sensor (I2C)
**Spezifikationen:**
- Messbereich Temperatur: -40 bis +85°C (±1°C Genauigkeit)
- Luftfeuchtigkeit: 0-100% RH (±3% Genauigkeit)
- Luftdruck: 300-1100 hPa (±1 hPa Genauigkeit)
- I2C-Adresse: 0x76

**Initialisierung:**
```cpp
bool initSensor() {
  Wire.begin(I2C_SDA, I2C_SCL);
  return bme.begin(BME280_ADDR);
}
```
**Messung:**
```cpp
float temp = bme.readTemperature();  // °C
float hum = bme.readHumidity();      // %
float pres = bme.readPressure() / 100.0F;  // hPa
```
### 5.4 SD-Karten-Logger

**Format:** CSV mit ISO 8601 Zeitstempeln (CET/CEST)

**Dateistruktur (/smarthome.csv):**
```
iso8601,epoch,temp_C,humidity_%,pressure_hPa
2025-11-30T14:23:45+01:00,1732972425,23.45,65.32,1013.25
2025-11-30T14:24:45+01:00,1732972485,23.47,65.28,1013.26
```
**Implementierung:**
```cpp
void appendCSV(time_t ts, float temp, float hum, float pres) {
  File file = SD.open(CSV_PATH, FILE_APPEND);
  if (!file) return;
  
  struct tm timeinfo;
  localtime_r(&ts, &timeinfo);
  
  char iso[32];
  strftime(iso, sizeof(iso), "%Y-%m-%dT%H:%M:%S%z", &timeinfo);
  
  file.printf("%s,%lu,%.2f,%.2f,%.2f\n", iso, ts, temp, hum, pres);
  file.close();
}
```
### 5.5 LED-Feedback-System
**PWM-Konfiguration:**
- Kanal: 0
- Frequenz: 5 kHz
- Auflösung: 8 Bit (0-255)

**Muster:**
|Pattern	|Beschreibung|	Code|
|-----------------|---------------------|---------------------------|
|LED_OFF|	Dauerhaft aus|- |
|LED_ON|	Dauerhaft an|- |
|LED_BLINK_SLOW|	500ms Toggle|	WiFi-Verbindung|
|LED_BLINK_FAST|	250ms Toggle|	MQTT-Fehler|
|LED_PULSE|	6x kurze Pulse|	MQTT-Publish|
|LED_BREATHE|	PWM Fade (Sinus)|	Warten auf Minute|

**Breathe-Effekt:**
```cpp
if (ledWifiPattern == LED_BREATHE) {
  pwmEnabled = true;
  float phase = (millis() % 2000) / 2000.0 * 2.0 * PI;
  int brightness = (sin(phase) + 1.0) * 127.5;
  ledcWrite(LED_PWM_CHANNEL, brightness);
}
```
### 5.6 Deep-Sleep-Management
**Dual-Wakeup-Quellen:**
1. Timer: Periodisches Aufwachen (konfigurierbar: 60s, 120s, 180s, ...)
2. Ext0 (GPIO14): Button-Wakeup für manuelle Konfiguration

**Sleep-Zeit-Berechnung (Minutensynchronisation):**
```cpp
uint32_t calculateSleepTime() {
  if (sleepSec % 60 != 0) return sleepSec;
  
  time_t now = time(nullptr);
  struct tm timeinfo;
  localtime_r(&now, &timeinfo);
  
  uint32_t nextSleep = (60 - timeinfo.tm_sec) + (sleepSec - WAKEUP_BUFFER_SEC);
  return nextSleep;
}
```

**Deep-Sleep-Aktivierung:**
```cpp
esp_sleep_enable_timer_wakeup(sleepSec * 1000000ULL);
esp_sleep_enable_ext0_wakeup(GPIO_NUM_14, 0);  // LOW = pressed
esp_deep_sleep_start();
```
### 5.7 NPT-Zeitsynchronisation

**Timezone:** CET (UTC+1) / CEST (UTC+2) mit automatischer Sommerzeitumstellung

**Konfiguration:**
```cpp
void syncTime() {
  configTime(3600, 3600, "pool.ntp.org");
  setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
  tzset();
  
  // Warten auf Sync
  time_t now = 0;
  while (now < 100000) {
    time(&now);
    delay(100);
  }
}
```
**DST-Regeln (Europa):**
- Sommerzeit: Letzter Sonntag im März, 02:00 Uhr
- Winterzeit: Letzter Sonntag im Oktober, 03:00 Uhr

---
## 6. Einrichtung und Nutzung
### 6.1 Hardware-Aufbau
**Schritt 1:** Komponenten verlöten
- LED Grün (GPIO17) + 220Ω Vorwiderstand → GND
- LED Rot (GPIO16) + 220Ω Vorwiderstand → GND
- Taster (GPIO14) → GND (interner Pull-Up aktiv)
- BME280: SDA → GPIO21, SCL → GPIO22, VCC → 3.3V, GND → GND
- SD-Modul: MOSI → GPIO23, MISO → GPIO19, SCK → GPIO18, CS → GPIO5, VCC → 5V, GND → GND

**Schritt 2:** Stromversorgung
- 18650 Akku in Halterung einsetzen
- 5V Step-Up Converter an VIN/GND des ESP32 anschließen

### 6.2 Software-Installation

**Voraussetzungen:**
- VS Code mit PlatformIO Extension
- Git (optional)

**Installation:**
```ini
# Repository klonen (oder ZIP herunterladen)
git clone <repository-url>
cd "SmartHome MCR"

# Bibliotheken installieren
pio lib install

# secrets.h anpassen
Copy-Item src\secrets.template.h src\secrets.h
# Editieren: MQTT-Credentials eintragen

# Kompilieren
pio run

# Flashen
pio run -t upload

# Serieller Monitor
pio device monitor
```
### 6.3 Erstinbetriebnahme
**Schritt 1:** Access Point verbinden
1. ESP32 mit Strom versorgen
2. WiFi-Netzwerk [ESP32-SSID] (Passwort: [ESP32-Passwort]) auf Smartphone/PC suchen
3. Verbinden (Captive Portal öffnet automatisch oder http://[ESP32-IP] aufrufen)

**Schritt 2:** WiFi konfigurieren
1. Im Web-Interface: "WiFi-Konfiguration" öffnen
2. Heimnetzwerk auswählen (oder SSID manuell eingeben)
3. Passwort eingeben → "Speichern und Neustart"

**Schritt 3:** MQTT konfigurieren
1. ESP32 verbindet sich mit Heimnetzwerk (IP im Serial Monitor)
2. Browser: http://[ESP32-IP]
3. "MQTT-Konfiguration" öffnen
4. EMQX-Credentials eintragen → "Speichern und Neustart"

**Schritt 4:** Deep-Sleep aktivieren
1. "Sleep-Einstellungen" öffnen
2. Intervall wählen (z.B. 60s für Minutentakt)
3. "Deep-Sleep aktiviert" → "Speichern und Neustart"

### 6.4 Bedienung
**Taster-Funktionen:**
- Kurzer Druck (Normal → Config): Wechselt zu Konfigurationsmodus (AP wird gestartet)
- Kurzer Druck (Config → Normal): Verlässt Konfigurationsmodus, kehrt zu Normalbetrieb zurück
- Wakeup aus Deep-Sleep: Startet Konfigurationsmodus

### 6.5 Web-Interface
**Dashboard (http://[ESP32-IP]):**
```
╔════════════════════════════════════════╗
║     ESP32 Umwelt-Datenlogger           ║
╠════════════════════════════════════════╣
║ Status: ✅ NORMAL                      ║
║ WiFi: ✅ Verbunden (SSID)              ║
║ MQTT: ✅ Verbunden                     ║
║ Sensor: ✅ BME280 OK                   ║
║ SD-Karte: ✅ Bereit                    ║
║ Sleep: ✅ Aktiviert (60s)              ║
╠════════════════════════════════════════╣
║ [WiFi-Config] [MQTT-Config]            ║
║ [Sleep-Config] [Jetzt Messen]          ║
║ [Neustart]                             ║
╚════════════════════════════════════════╝
```
---
## 7.  Herausforderungen und Lösungen
### 7.1 Problem: MQTT Offline-Status
**Herausforderung:**
"offline"-Nachricht erschien gleichzeitig mit "online" nach Wakeup, nicht unmittelbar vor Deep-Sleep.

**Root Cause:**
Last Will Testament wird nur bei unerwarteter Trennung gesendet, nicht bei sauberem Disconnect.

**Lösung:**
```cpp
// Explizite offline-Nachricht vor Sleep
mqtt.publish("smarthome/senderniklas/status", "offline", true);
delay(100);  // Sicherstellen, dass Nachricht gesendet wird
mqtt.disconnect();
delay(50);
WiFi.disconnect(true);
```
### 7.2 Problem: Minutensynchronisation
**Herausforderung:**
Messungen sollten genau zur vollen Minute erfolgen (12:21:00, 12:22:00), nicht versetzt.

**Lösung:**
Dynamische Sleep-Zeit-Berechnung:
```cpp
uint32_t nextSleep = (60 - currentSeconds) + (sleepSec - WAKEUP_BUFFER_SEC);
```
- Wartet bis volle Minute (mit waitForFullMinute())
- Sleep-Zeit wird um Wakeup-Buffer (7s) reduziert
- Nur aktiv bei Intervallen die Vielfache von 60s sind
  
### 7.3 Problem: Button während Wartezeit
**Herausforderung:**
Taster funktionierte nicht während waitForFullMinute() (blockierendes delay).

**Lösung:**
Non-blocking Wait-Loop mit Button-Polling:
```cpp
void waitForFullMinute() {
  while (timeinfo.tm_sec != 0) {
    handleButton();  // ← Button-Check in Loop
    if (buttonPressed) return;  // Sofortiger Abbruch
    delay(100);
    time_t now = time(nullptr);
    localtime_r(&now, &timeinfo);
  }
}
```
### 7.4 Problem: Energieverbrauch
**Herausforderung:**
Initiales Design hatte 2-3 Tage Laufzeit, Ziel war ~1 Jahr.

**Lösung:**
1. Deep-Sleep: Reduktion von 80 mA (Idle) auf 10 µA
2. LED-Optimierung: LEDs nur im Normalbetrieb aktiv, PWM-Dimming
3. WiFi-Power-Save: Modem-Sleep aktiviert
4. Schnelles Connect: Timeout auf 15s reduziert
5. MQTT QoS 0: Keine ACK-Wartezeiten

**Resultat:** 401 Tage Laufzeit (theoretisch)

---
## 8.  Verwendete Quellen und Ressourcen
### 8.1 Offizielle Dokumentation
|Quelle|	URL|
|-----------------|---------------------|
|ESP32 Arduino Core|	https://docs.espressif.com/projects/arduino-esp32/|
|PlatformIO Docs|	https://docs.platformio.org/|
|MQTT Specification|	https://mqtt.org/mqtt-specification/|
|BME280 Datasheet|	https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf|
|EMQX Cloud|	https://www.emqx.com/en/cloud|

### 8.2 Bibliotheken-Repositories
|Bibliothek|	Repository|
|-----------------|---------------------|
|PubSubClient|	https://github.com/knolleary/pubsubclient|
|ArduinoJson|	https://github.com/bblanchon/ArduinoJson|
|Adafruit BME280|	https://github.com/adafruit/Adafruit_BME280_Library|

### 8.3 Tutorials & Hilfen
- **ESP32 Deep-Sleep Tutorial:** [RandomNerdTutorials.com](https://randomnerdtutorials.com/esp32-deep-sleep-arduino-ide-wake-up-sources/)
- **MQTT over TLS:** ESP32.com Forum
- **NTP Timezone (CET):** Time.h Documentation (GNU LibC)
- **PWM LED Dimming:** ESP32 LEDC Examples

### 8.4 Tools & Software
|Tool|	Version|	Zweck|
|-----------------|---------------------|---------------------|
|VS Code|	1.85|	IDE|
|PlatformIO|	6.1|	Build-System|
|Fusion 360|	2024|	CAD (Gehäuse)|
|MQTT Explorer|	0.4.0-beta|	MQTT-Debugging|
|Serial Monitor|	0.4.0|	Log-Analyse|

### 8.5 Hardware-Recources
- ESP32-WROOM-32D Datasheet (Espressif)
- BME280 Digital Humidity, Pressure and Temperature Sensor (Bosch)
- 18650 Li-Ion Cell Specification (Samsung INR18650-25R)

## 9.  Anhang
### 9.1 Konfigurationsdatei (secrets.h)
```cpp
#pragma once

static const char* DEFAULT_WIFI_SSID = "Default_SSID";
static const char* DEFAULT_WIFI_PASS = "Default_Passwort";

static const char* DEFAULT_MQTT_HOST = "default.emqxsl.com";
static const uint16_t DEFAULT_MQTT_PORT = 8883;
static const char* DEFAULT_MQTT_USER = "Default_User";
static const char* DEFAULT_MQTT_PASS = "Default_Passwort";

static const char ROOT_CA[] PROGMEM = R"PEM(
-----BEGIN CERTIFICATE-----
[DigiCert Global Root CA]
-----END CERTIFICATE-----
)PEM";
```

### 9.2 PlatformIO-Konfiguration
```
[env:az-delivery-devkit-v4]
platform = espressif32
board = az-delivery-devkit-v4
framework = arduino
monitor_speed = 115200

lib_deps =
    knolleary/PubSubClient @ ^2.8
    bblanchon/ArduinoJson @ ^6
    adafruit/Adafruit BME280 Library
    adafruit/Adafruit Unified Sensor

build_flags =
    -D CORE_DEBUG_LEVEL=3
```

### 9.3 CSV-Beispieldaten
```
iso8601,epoch,temp_C,humidity_%,pressure_hPa
2025-11-30T14:20:00+01:00,1732972800,22.34,64.21,1013.45
2025-11-30T14:21:00+01:00,1732972860,22.35,64.18,1013.46
2025-11-30T14:22:00+01:00,1732972920,22.36,64.15,1013.47
```

### 9.4 MQTT-Payload-Beispiele
Status:
```
Topic: smarthome/senderlukas/status
Payload: "online"
Retained: true
```

State:
```json
{
  "state": "NORMAL",
  "timestamp": 1732972800,
  "description": "Normalbetrieb aktiv",
  "sleep_enabled": true,
  "sleep_interval_sec": 60,
  "wifi_connected": true,
  "mqtt_connected": true,
  "sd_ok": true,
  "sensor_ok": true
}
```

Environment (env):
```json
{
  "temperature_C": 22.34,
  "humidity_%": 64.21,
  "pressure_hPa": 1013.45,
  "timestamp": 1732972800,
  "iso8601": "2025-11-30T14:20:00+01:00"
}
```

### 9.5 Pinbelegung (Referenz)
|GPIO|	Funktion|	Richtung|	Bemerkung|
|-----------------|---------------------|---------------------|---------------------|
|14|	Button|	Input|	Pull-Up aktiv|
|16|	LED Rot|	Output|	Digital|
|17|	LED Grün| Output|	PWM (LEDC Ch0)|
|18|	SPI SCK|	Output|	SD-Karte|
|19|	SPI MISO|	Input|	SD-Karte|
|21|	I2C SDA|	I/O|	BME280|
|22|	I2C SCL|	Output|	BME280|
|23|	SPI MOSI|	Output|	SD-Karte|
|5|	SD CS|	Output|	Chip Select|
