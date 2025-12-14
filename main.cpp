/*
=============================================================================
ESP32 BME280 UMWELT-DATENLOGGER MIT MQTT & DEEP-SLEEP
=============================================================================
Beschreibung:
  Batteriebetriebener Datenlogger für Umweltdaten mit Cloud-Anbindung.
  Optimiert für niedrigen Stromverbrauch durch Deep-Sleep zwischen Messungen.
  Robuste Fehlerbehandlung mit automatischer Recovery.

Hardware:
  ┌───────────────────────────────────────────────────────────────────┐
  │ ESP32 DevKit V1                                                   │
  │  ├─ GPIO14 ─── Taster ─────────────── GND    (Config-Mode Toggle) │
  │  ├─ GPIO17 ─── LED Grün ──── 220Ω ─┬─ GND  (WiFi/MQTT Status)     │
  │  ├─ GPIO16 ─── LED Rot  ──── 220Ω  ┘                              │
  │  ├─ GPIO21 ─── I2C SDA ────────────── BME280 (0x76)               │
  │  ├─ GPIO22 ─── I2C SCL ───────────────┘                           │
  │  ├─ GPIO5  ─── SD CS ──────────────── SD-Karten Modul             │
  │  ├─ GPIO23 ─── SPI MOSI ──────────────┘││                         │
  │  ├─ GPIO19 ─── SPI MISO ───────────────┘│                         │
  │  └─ GPIO18 ─── SPI SCK ─────────────────┘                         │
  └───────────────────────────────────────────────────────────────────┘

Features:
  ✅ Automatische WiFi-Verbindung mit Config-AP Fallback
  ✅ Sichere MQTT über TLS (EMQX Cloud)
  ✅ Minutengenaue Zeitsteuerung (60s/120s/180s Intervalle)
  ✅ Deep-Sleep mit Timer & Button-Wakeup
  ✅ Lokale CSV-Speicherung (ISO 8601 Format, CET/CEST)
  ✅ Web-Interface für Konfiguration
  ✅ State Machine mit automatischer Fehlerbehandlung
  ✅ NTP-Zeitsynchronisation mit EU-Sommerzeit

Stromverbrauch (ca.):
  - Deep-Sleep:            ~10 µA
  - Aktiv (WiFi):          ~160 mA
  - Messung:               ~1s bei 60s Intervall = 0.27 mAh pro Stunde
  - 2600mAh 18650 Akku:    ~400 Tage Laufzeit

MQTT Topics:
  📤 smarthome/senderlukas/status  → "online"/"offline" (retained)
  📤 smarthome/senderlukas/state   → System-Status JSON (retained)
  📤 smarthome/senderlukas/env     → Messdaten JSON

CSV-Format (SD-Karte):
  iso8601,epoch,temp_C,humidity_%,pressure_hPa
  2025-11-30T14:23:45+01:00,1732972425,23.45,65.32,1013.25

Web-Interface:
  - Config-AP:  http://192.168.4.1
  - Station:    http://<ESP32-IP>
  - SSID:       "ESP32-Setup"
  - Passwort:   "smarthome"

Bedienung:
  🔘 Taster kurz drücken: Toggle Normal ↔ Config
  🔘 Taster beim Boot:    Startet Config-Mode
  🔘 Deep-Sleep Wakeup:   Timer oder Taster

LED-Codes:
  State               | Grün            | Rot
  ────────────────────┼─────────────────┼──────────────────
  INIT                | Blinkt langsam  | Aus
  NORMAL              | An              | Aus
  CONFIG_MANUAL       | An              | Blinkt langsam
  ERROR_WIFI          | Aus             | Blinkt schnell
  ERROR_MQTT          | Blinkt schnell  | An
  ERROR_OTHER         | Aus             | An
  Warte auf Minute    | Breathe (dimmt) | Aus
  MQTT Publish        | 6x Pulse        | Aus

Entwickler: Lukas Reif
Datum: 2025-11-30
=============================================================================
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WebServer.h>
#include <Preferences.h>
#include <time.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Adafruit_BME280.h>
#include <SPI.h>
#include <SD.h>
#include "secrets.h"  // ROOT_CA, DEFAULT_WIFI_SSID, etc.

// =============================================================================
// HARDWARE-PINS
// =============================================================================
#define BTN_PIN       4    // Taster (intern Pull-Up)
#define LED_WIFI      17    // Grüne LED (WiFi/MQTT)
#define LED_ERROR     16    // Rote LED (Fehler)
#define I2C_SDA       21    // BME280 Data
#define I2C_SCL       22    // BME280 Clock
#define SD_CS         5     // SD-Karte CS
#define BME280_ADDR   0x76  // I2C Adresse (alternativ: 0x77)

// PWM für LED-Dimming
#define LED_PWM_CHANNEL     0
#define LED_PWM_FREQ        5000
#define LED_PWM_RESOLUTION  8  // 0-255

// =============================================================================
// KONFIGURATION
// =============================================================================
const char* AP_SSID = "ESP32-Setup-Niklas";
const char* AP_PASS = "smarthome";
const char* CSV_PATH = "/smarthome.csv";

// Zeitintervalle
const uint32_t DEFAULT_SLEEP_SEC = 60;
const uint32_t MEASURE_INTERVAL_MS = 60000;
const uint32_t CONFIG_TIMEOUT_MS = 300000;
const uint32_t DEEP_SLEEP_TIMEOUT_MS = 600000;
const uint32_t WAKEUP_BUFFER_SEC = 7;
const uint32_t STATE_PUBLISH_INTERVAL = 60000;

// Hardware
const uint32_t DEBOUNCE_MS = 50;
const uint32_t SD_CHECK_INTERVAL = 10000;

// =============================================================================
// ENUMS
// =============================================================================
enum SystemState {
  STATE_INIT,
  STATE_NORMAL,
  STATE_CONFIG_MANUAL,
  STATE_ERROR_WIFI,
  STATE_ERROR_MQTT,
  STATE_ERROR_OTHER,
  STATE_DEEP_SLEEP_ONLY
};

enum LEDPattern {
  LED_OFF,
  LED_ON,
  LED_BLINK_SLOW,   // 500ms
  LED_BLINK_FAST,   // 250ms
  LED_PULSE,        // 6x kurze Pulse
  LED_BREATHE       // PWM Dimming
};

// =============================================================================
// STRUKTUREN
// =============================================================================
struct SystemErrors {
  bool wifi, mqtt, sd, sensor;
  
  void reset() { wifi = mqtt = sd = sensor = false; }
  bool hasError() const { return wifi || mqtt || sd || sensor; }
  
  String toString() const {
    String s;
    if (wifi) s += "WiFi | ";
    if (mqtt) s += "MQTT | ";
    if (sd) s += "SD | ";
    if (sensor) s += "Sensor | ";
    return s.length() > 3 ? s.substring(0, s.length() - 3) : s;
  }
};

// =============================================================================
// GLOBALE VARIABLEN
// =============================================================================
// Hardware
Preferences prefs;
WebServer server(80);
WiFiClientSecure tlsClient;
PubSubClient mqtt(tlsClient);
Adafruit_BME280 bme;

// Status
SystemState currentState = STATE_INIT;
SystemErrors errors;
unsigned long stateStartTime = 0;
unsigned long lastMeasure = 0;
unsigned long lastSDCheck = 0;
unsigned long lastStatePublish = 0;

// Flags
bool apActive = false;
bool httpStarted = false;
bool wifiConnected = false;
bool mqttConnected = false;
bool useDeepSleep = true;
bool sdOK = false;
bool bmeInitialized = false;
bool buttonPressed = false;

// Button
int btnLast = HIGH, btnStable = HIGH;
unsigned long btnChangeMs = 0;

// Config
String wifiSsid, wifiPass, mqttHost, mqttUser, mqttPass;
uint16_t mqttPort = 8883;
uint32_t sleepSec = DEFAULT_SLEEP_SEC;

// LED
LEDPattern ledWifiPattern = LED_OFF;
LEDPattern ledErrorPattern = LED_OFF;
unsigned long lastLedUpdate = 0;
bool ledState = false;
int pulseCount = 0;
bool pwmEnabled = false;

// =============================================================================
// HILFSFUNKTIONEN
// =============================================================================

const char* getStateName(SystemState state) {
  const char* names[] = {"INIT", "NORMAL", "CONFIG", "ERR_WIFI", "ERR_MQTT", "ERR_OTHER", "SLEEP_ONLY"};
  return (state <= STATE_DEEP_SLEEP_ONLY) ? names[state] : "UNKNOWN";
}

// =============================================================================
// VORWÄRTSDEKLARATIONEN
// =============================================================================
void handleButton();
void updateLEDs();
void startHTTP();
void startAP();
void transitionToState(SystemState newState);
void publishState(SystemState state);

/**
 * Published aktuellen System-Status als JSON zu MQTT
 * Topic: smarthome/senderlukas/state (retained)
 * 
 * Payload-Beispiel:
 * {
 *   "state": "NORMAL",
 *   "timestamp": 1732972425,
 *   "description": "Normalbetrieb aktiv",
 *   "sleep_enabled": true,
 *   "sleep_interval_sec": 60,
 *   "wifi_connected": true,
 *   "mqtt_connected": true,
 *   "sd_ok": true,
 *   "sensor_ok": true
 * }
 */
void publishStateToMQTT(SystemState state) {
  if (!mqttConnected) return;
  
  StaticJsonDocument<256> doc;
  doc["state"] = getStateName(state);
  doc["timestamp"] = (uint32_t)time(nullptr);
  
  // State-spezifische Informationen
  switch (state) {
    case STATE_INIT:
      doc["description"] = "Systeminitialisierung läuft";
      break;
    case STATE_NORMAL:
      doc["description"] = "Normalbetrieb aktiv";
      doc["sleep_enabled"] = useDeepSleep;
      doc["sleep_interval_sec"] = sleepSec;
      break;
    case STATE_CONFIG_MANUAL:
      doc["description"] = "Manuelle Konfiguration";
      doc["ap_active"] = true;
      break;
    case STATE_ERROR_WIFI:
      doc["description"] = "WiFi-Fehler";
      doc["error"] = "WiFi-Verbindung fehlgeschlagen";
      break;
    case STATE_ERROR_MQTT:
      doc["description"] = "MQTT-Fehler";
      doc["error"] = "MQTT-Verbindung fehlgeschlagen";
      break;
    case STATE_ERROR_OTHER:
      doc["description"] = "Sonstiger Fehler";
      doc["error"] = errors.toString();
      break;
    case STATE_DEEP_SLEEP_ONLY:
      doc["description"] = "Notfall-Sleep";
      doc["error"] = "Nur per Reset aufweckbar";
      break;
  }
  
  // System-Status
  doc["wifi_connected"] = wifiConnected;
  doc["mqtt_connected"] = mqttConnected;
  doc["sd_ok"] = sdOK;
  doc["sensor_ok"] = bmeInitialized;
  
  char payload[384];
  serializeJson(doc, payload);
  
  mqtt.publish("smarthome/senderniklas/state", payload, true);
  Serial.printf("📡 State published: %s\n", payload);
}

// =============================================================================
// LED-STEUERUNG
// =============================================================================

void setLEDPattern(SystemState state) {
  const LEDPattern patterns[][2] = {
    {LED_BLINK_SLOW, LED_OFF},      // INIT
    {LED_ON, LED_OFF},              // NORMAL
    {LED_ON, LED_BLINK_SLOW},       // CONFIG
    {LED_OFF, LED_BLINK_FAST},      // ERROR_WIFI
    {LED_BLINK_FAST, LED_ON},       // ERROR_MQTT
    {LED_OFF, LED_ON},              // ERROR_OTHER
    {LED_OFF, LED_OFF}              // SLEEP
  };
  
  if (state <= STATE_DEEP_SLEEP_ONLY) {
    ledWifiPattern = patterns[state][0];
    ledErrorPattern = patterns[state][1];
  }
}

void updateLEDs() {
  unsigned long now = millis();
  
  // WiFi LED (mit PWM-Unterstützung)
  if (ledWifiPattern == LED_BREATHE) {
    if (!pwmEnabled) {
      ledcSetup(LED_PWM_CHANNEL, LED_PWM_FREQ, LED_PWM_RESOLUTION);
      ledcAttachPin(LED_WIFI, LED_PWM_CHANNEL);
      pwmEnabled = true;
    }
    float brightness = (sin((now % 2000) / 2000.0 * 2.0 * PI) + 1.0) / 2.0;
    ledcWrite(LED_PWM_CHANNEL, (uint8_t)(brightness * 255));
  } else {
    if (pwmEnabled) {
      ledcDetachPin(LED_WIFI);
      pinMode(LED_WIFI, OUTPUT);
      pwmEnabled = false;
    }
    
    switch (ledWifiPattern) {
      case LED_OFF: 
        digitalWrite(LED_WIFI, LOW); 
        break;
      case LED_ON: 
        digitalWrite(LED_WIFI, HIGH); 
        break;
      case LED_BLINK_SLOW:
        if (now - lastLedUpdate > 500) { 
          ledState = !ledState; 
          digitalWrite(LED_WIFI, ledState); 
          lastLedUpdate = now; 
        }
        break;
      case LED_BLINK_FAST:
        if (now - lastLedUpdate > 250) { 
          ledState = !ledState; 
          digitalWrite(LED_WIFI, ledState); 
          lastLedUpdate = now; 
        }
        break;
      case LED_PULSE:
        if (pulseCount > 0 && now - lastLedUpdate > 100) {
          ledState = !ledState;
          digitalWrite(LED_WIFI, ledState);
          lastLedUpdate = now;
          if (!ledState) pulseCount--;
        }
        if (pulseCount == 0) ledWifiPattern = LED_ON;
        break;
      default: 
        break;
    }
  }
  
  // Error LED (nur digital)
  switch (ledErrorPattern) {
    case LED_OFF: 
      digitalWrite(LED_ERROR, LOW); 
      break;
    case LED_ON: 
      digitalWrite(LED_ERROR, HIGH); 
      break;
    case LED_BLINK_SLOW:
      if (now - lastLedUpdate > 500) { 
        digitalWrite(LED_ERROR, ledState); 
      }
      break;
    case LED_BLINK_FAST:
      if (now - lastLedUpdate > 250) { 
        digitalWrite(LED_ERROR, ledState); 
      }
      break;
    default: 
      break;
  }
}

void signalPublish() {
  ledWifiPattern = LED_PULSE;
  pulseCount = 6;
}

// =============================================================================
// NVS-SPEICHER
// =============================================================================

/**
 * Lädt alle Konfigurationsparameter aus dem NVS-Speicher
 * Verwendet Standardwerte aus secrets.h falls keine Werte gespeichert sind
 */
void loadConfig() {
  prefs.begin("config", true);
  wifiSsid     = prefs.getString("wifiSsid", DEFAULT_WIFI_SSID);
  wifiPass     = prefs.getString("wifiPass", DEFAULT_WIFI_PASS);
  mqttHost     = prefs.getString("mqttHost", DEFAULT_MQTT_HOST);
  mqttPort     = prefs.getUInt("mqttPort", DEFAULT_MQTT_PORT);
  mqttUser     = prefs.getString("mqttUser", DEFAULT_MQTT_USER);
  mqttPass     = prefs.getString("mqttPass", DEFAULT_MQTT_PASS);
  sleepSec     = prefs.getUInt("sleepSec", DEFAULT_SLEEP_SEC);
  useDeepSleep = prefs.getBool("deepSleep", true);
  prefs.end();
}

/**
 * Speichert String-Wert in NVS
 * @param key NVS-Schlüssel
 * @param value String-Wert
 */
void saveConfigString(const char* key, const String& value) {
  prefs.begin("config", false);
  prefs.putString(key, value);
  prefs.end();
}

/**
 * Speichert uint32_t-Wert in NVS
 * @param key NVS-Schlüssel
 * @param value Ganzzahl-Wert
 */
void saveConfigUInt(const char* key, uint32_t value) {
  prefs.begin("config", false);
  prefs.putUInt(key, value);
  prefs.end();
}

/**
 * Speichert bool-Wert in NVS
 * @param key NVS-Schlüssel
 * @param value Boolean-Wert
 */
void saveConfigBool(const char* key, bool value) {
  prefs.begin("config", false);
  prefs.putBool(key, value);
  prefs.end();
}

// =============================================================================
// WIFI
// =============================================================================

bool connectWiFi(uint32_t timeoutMs = 15000) {
  if (wifiSsid.isEmpty()) return false;
  if (WiFi.status() == WL_CONNECTED) { wifiConnected = true; return true; }
  
  Serial.printf("📶 WiFi: %s\n", wifiSsid.c_str());
  WiFi.mode(apActive ? WIFI_AP_STA : WIFI_STA);
  WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());
  
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
    delay(100);
    updateLEDs();
  }
  
  wifiConnected = (WiFi.status() == WL_CONNECTED);
  errors.wifi = !wifiConnected;
  
  if (wifiConnected) {
    Serial.printf("✅ WiFi: %s (%d dBm)\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
  } else {
    Serial.println("❌ WiFi failed");
  }
  
  return wifiConnected;
}

void startAP() {
  if (apActive) return;
  WiFi.mode(WIFI_AP_STA);
  apActive = WiFi.softAP(AP_SSID, AP_PASS);
  if (apActive) Serial.printf("📡 AP: %s @ %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
}

void stopAP() {
  if (!apActive) return;
  WiFi.softAPdisconnect(true);
  apActive = false;
  Serial.println("🛑 AP stopped");
}

// =============================================================================
// MQTT
// =============================================================================

bool connectMQTT() {
  if (!wifiConnected || mqttHost.isEmpty()) { errors.mqtt = true; return false; }
  if (mqtt.connected()) { mqttConnected = true; errors.mqtt = false; return true; }
  
  Serial.printf("🔐 MQTT: %s:%d\n", mqttHost.c_str(), mqttPort);
  
  tlsClient.setCACert(ROOT_CA);
  tlsClient.setTimeout(10);
  mqtt.setServer(mqttHost.c_str(), mqttPort);
  mqtt.setKeepAlive(60);
  mqtt.setSocketTimeout(10);
  mqtt.setBufferSize(512);
  
  String cid = "esp32-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  mqttConnected = mqtt.connect(cid.c_str(), mqttUser.c_str(), mqttPass.c_str(),
                               "smarthome/senderniklas/status", 1, true, "offline");
  
  if (mqttConnected) {
    mqtt.publish("smarthome/senderniklas/status", "online", true);
    Serial.println("✅ MQTT OK");
    errors.mqtt = false;
  } else {
    Serial.printf("❌ MQTT error: %d\n", mqtt.state());
    errors.mqtt = true;
  }
  
  return mqttConnected;
}

void publishState(SystemState state) {
  if (!mqttConnected) return;
  
  StaticJsonDocument<256> doc;
  doc["state"] = getStateName(state);
  doc["ts"] = time(nullptr);
  doc["wifi"] = wifiConnected;
  doc["mqtt"] = mqttConnected;
  doc["sd"] = sdOK;
  doc["sensor"] = bmeInitialized;
  
  if (state == STATE_NORMAL) {
    doc["sleep"] = useDeepSleep;
    doc["interval"] = sleepSec;
  } else if (errors.hasError()) {
    doc["error"] = errors.toString();
  }
  
  char buf[384];
  serializeJson(doc, buf);
  mqtt.publish("smarthome/senderniklas/state", buf, true);
}

// =============================================================================
// ZEIT (CET/CEST)
// =============================================================================

void syncTime() {
  if (!wifiConnected) return;
  static bool synced = false;
  if (synced) return;
  
  Serial.println("🕐 NTP sync...");
  configTime(3600, 3600, "pool.ntp.org", "time.nist.gov");
  setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
  tzset();
  
  time_t now = 0;
  for (int i = 0; i < 30 && now < 1700000000; i++) {
    delay(200);
    now = time(nullptr);
  }
  
  synced = (now >= 1700000000);
  if (synced) {
    char buf[64];
    struct tm t;
    localtime_r(&now, &t);
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S %Z", &t);
    Serial.printf("✅ Time: %s\n", buf);
  }
}

uint32_t calculateSleepTime() {
  bool sync = (sleepSec >= 60 && sleepSec % 60 == 0);
  if (!sync) return sleepSec;
  
  time_t now = time(nullptr);
  if (now < 1700000000) return sleepSec;
  
  struct tm t;
  localtime_r(&now, &t);
  uint32_t secToMinute = (60 - t.tm_sec) % 60;
  uint32_t sleep = secToMinute + sleepSec - WAKEUP_BUFFER_SEC;
  
  Serial.printf("⏰ Sleep: %us (sync @:%02d)\n", sleep, t.tm_sec);
  return (sleep < WAKEUP_BUFFER_SEC) ? sleepSec - WAKEUP_BUFFER_SEC : sleep;
}

void waitForFullMinute() {
  if (sleepSec < 60 || sleepSec % 60 != 0) return;
  
  time_t now = time(nullptr);
  if (now < 1700000000) return;
  
  struct tm t;
  localtime_r(&now, &t);
  if (t.tm_sec == 0) return;
  
  uint32_t waitMs = (60 - t.tm_sec) * 1000;
  Serial.printf("⏳ Wait %us to full minute\n", waitMs/1000);
  
  LEDPattern prev = ledWifiPattern;
  ledWifiPattern = LED_BREATHE;
  buttonPressed = false;
  
  unsigned long start = millis();
  while (millis() - start < waitMs) {
    handleButton();
    updateLEDs();
    if (buttonPressed) break;
    delay(10);
  }
  
  ledWifiPattern = prev;
  if (pwmEnabled) {
    ledcDetachPin(LED_WIFI);
    pinMode(LED_WIFI, OUTPUT);
    pwmEnabled = false;
  }
}

// =============================================================================
// SD-KARTE
// =============================================================================

bool initSD() {
  sdOK = SD.begin(SD_CS);
  if (!sdOK) { errors.sd = true; return false; }
  
  if (!SD.exists(CSV_PATH)) {
    File f = SD.open(CSV_PATH, FILE_WRITE);
    if (!f) { sdOK = false; errors.sd = true; return false; }
    f.println("iso8601,epoch,temp_C,humidity_%,pressure_hPa");
    f.close();
  }
  
  errors.sd = false;
  return true;
}

void checkSD() {
  if (millis() - lastSDCheck < SD_CHECK_INTERVAL) return;
  lastSDCheck = millis();
  bool was = sdOK;
  sdOK = SD.exists(CSV_PATH);
  if (was != sdOK) Serial.printf("SD: %s\n", sdOK ? "OK" : "FAIL");
}

void appendCSV(time_t ts, float temp, float hum, float pres) {
  if (!sdOK) return;
  File f = SD.open(CSV_PATH, FILE_APPEND);
  if (!f) { sdOK = false; return; }
  
  struct tm t;
  localtime_r(&ts, &t);
  char iso[32];
  strftime(iso, sizeof(iso), "%Y-%m-%dT%H:%M:%S%z", &t);
  String s = String(iso);
  if (s.length() > 2) s = s.substring(0, s.length()-2) + ":" + s.substring(s.length()-2);
  
  f.printf("%s,%lu,%.2f,%.2f,%.2f\n", s.c_str(), ts, temp, hum, pres);
  f.close();
}

// =============================================================================
// SENSOR
// =============================================================================

bool initSensor() {
  if (bmeInitialized) return true;
  bmeInitialized = bme.begin(BME280_ADDR);
  errors.sensor = !bmeInitialized;
  return bmeInitialized;
}

bool measureAndPublish() {
  if (!initSensor()) return false;
  
  float temp = bme.readTemperature();
  float hum = bme.readHumidity();
  float pres = bme.readPressure() / 100.0f;
  time_t ts = time(nullptr);
  
  if (isnan(temp) || isnan(hum) || isnan(pres)) {
    errors.sensor = true;
    return false;
  }
  
  Serial.printf("📊 T=%.1f°C H=%.1f%% P=%.1fhPa\n", temp, hum, pres);
  
  StaticJsonDocument<256> doc;
  doc["ts"] = ts;
  doc["temp"] = temp;
  doc["hum"] = hum;
  doc["pres"] = pres;
  
  char buf[256];
  serializeJson(doc, buf);
  
  bool ok = mqtt.publish("smarthome/senderniklas/env", buf);
  if (ok) signalPublish();
  else { mqttConnected = false; errors.mqtt = true; }
  
  if (sdOK) appendCSV(ts, temp, hum, pres);
  
  return ok;
}

// =============================================================================
// BUTTON
// =============================================================================

void handleButton() {
  int r = digitalRead(BTN_PIN);
  if (r != btnLast) btnChangeMs = millis();
  
  if (millis() - btnChangeMs > DEBOUNCE_MS && r != btnStable) {
    btnStable = r;
    if (btnStable == LOW) {
      buttonPressed = true;
      if (currentState == STATE_NORMAL) {
        transitionToState(STATE_CONFIG_MANUAL);
        startAP();
        startHTTP();
      } else if (currentState == STATE_CONFIG_MANUAL) {
        stopAP();
        transitionToState(STATE_NORMAL);
        lastMeasure = millis();
      }
    }
  }
  btnLast = r;
}

// =============================================================================
// WEB-INTERFACE (Stark vereinfacht)
// =============================================================================

String htmlHeader(const char* title) {
  return "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width'/>"
         "<title>" + String(title) + "</title><style>"
         "body{font-family:Arial;margin:20px;background:#f0f0f0}"
         ".card{background:white;padding:20px;border-radius:8px;margin:10px 0;box-shadow:0 2px 4px rgba(0,0,0,0.1)}"
         ".ok{color:green}.err{color:red}a{color:#06c;text-decoration:none}"
         "button{background:#06c;color:white;border:none;padding:10px 20px;border-radius:4px;cursor:pointer}"
         "button:hover{background:#048}input{width:100%;padding:8px;margin:5px 0;border:1px solid #ccc;border-radius:4px}"
         "</style></head><body>";
}

String htmlFooter() {
  return "<p style='text-align:center;color:#999;margin-top:30px'>ESP32 Logger v1.1</p></body></html>";
}

void handleRoot() {
  String html = htmlHeader("Dashboard");
  html += "<h2> System Dashboard</h2>";
  html += "<div class='card'><b>Status:</b> " + String(getStateName(currentState)) + "</div>";
  html += "<div class='card'><b>WiFi:</b> <span class='" + String(wifiConnected?"ok":"err") + "'>";
  html += wifiConnected ? wifiSsid + " (" + WiFi.localIP().toString() + ")" : "Disconnected";
  html += "</span></div>";
  html += "<div class='card'><b>MQTT:</b> <span class='" + String(mqttConnected?"ok":"err") + "'>";
  html += mqttConnected ? "Connected" : "Disconnected";
  html += "</span></div>";
  html += "<div class='card'><b>SD:</b> <span class='" + String(sdOK?"ok":"err") + "'>" + String(sdOK?"OK":"FAIL") + "</span></div>";
  html += "<div class='card'><b>Sensor:</b> <span class='" + String(bmeInitialized?"ok":"err") + "'>" + String(bmeInitialized?"OK":"FAIL") + "</span></div>";
  if (errors.hasError()) html += "<div class='card' style='background:#fee'><b>Errors:</b> " + errors.toString() + "</div>";
  html += "<div class='card'><a href='/wifi'>WiFi Config</a> | <a href='/mqtt'>MQTT Config</a> | <a href='/sleep'>Sleep Config</a></div>";
  html += "<div class='card'><a href='/measure'> Measure Now</a> | <a href='/restart'> Restart</a></div>";
  html += htmlFooter();
  server.send(200, "text/html", html);
}

void handleWiFi() {
  String html = htmlHeader("WiFi");
  html += "<h2> WiFi Configuration</h2><form method='POST' action='/wifi/save'>";
  html += "SSID:<br><input name='ssid' value='" + wifiSsid + "'/><br>";
  html += "Password:<br><input name='pass' type='password' placeholder='(leave empty to keep)'/><br><br>";
  html += "<button>Save</button></form><p><a href='/'> Back</a></p>";
  html += htmlFooter();
  server.send(200, "text/html", html);
}

void handleWiFiSave() {
  String ssid = server.arg("ssid");
  String pass = server.arg("pass");
  if (ssid.isEmpty()) { server.send(400, "text/plain", "SSID required"); return; }
  if (pass.isEmpty()) pass = wifiPass;
  
  wifiSsid = ssid;
  wifiPass = pass;
  saveConfigString("wifiSsid", ssid);
  saveConfigString("wifiPass", pass);
  
  server.send(200, "text/plain", "Saved. Reconnecting...");
  delay(500);
  WiFi.disconnect();
  transitionToState(STATE_INIT);
}

void handleMQTT() {
  String html = htmlHeader("MQTT");
  html += "<h2> MQTT Configuration</h2><form method='POST' action='/mqtt/save'>";
  html += "Host:<br><input name='host' value='" + mqttHost + "'/><br>";
  html += "Port:<br><input name='port' type='number' value='" + String(mqttPort) + "'/><br>";
  html += "User:<br><input name='user' value='" + mqttUser + "'/><br>";
  html += "Password:<br><input name='pass' type='password' placeholder='(leave empty to keep)'/><br><br>";
  html += "<button>Save</button></form><p><a href='/'> Back</a></p>";
  html += htmlFooter();
  server.send(200, "text/html", html);
}

void handleMQTTSave() {
  String host = server.arg("host");
  uint16_t port = server.arg("port").toInt();
  String user = server.arg("user");
  String pass = server.arg("pass");
  
  if (host.isEmpty() || !port) { server.send(400, "text/plain", "Invalid input"); return; }
  if (pass.isEmpty()) pass = mqttPass;
  
  mqttHost = host;
  mqttPort = port;
  mqttUser = user;
  mqttPass = pass;
  saveConfigString("mqttHost", host);
  saveConfigUInt("mqttPort", port);
  saveConfigString("mqttUser", user);
  saveConfigString("mqttPass", pass);
  
  server.send(200, "text/plain", "Saved. Reconnecting...");
  delay(500);
  mqtt.disconnect();
  transitionToState(STATE_INIT);
}

void handleSleep() {
  String html = htmlHeader("Sleep");
  html += "<h2> Sleep Configuration</h2><form method='POST' action='/sleep/save'>";
  html += "Interval (seconds):<br><input name='sec' type='number' value='" + String(sleepSec) + "' min='10'/><br><br>";
  html += "<input type='radio' name='m' value='d'" + String(useDeepSleep?" checked":"") + "/> Deep-Sleep<br>";
  html += "<input type='radio' name='m' value='n'" + String(!useDeepSleep?" checked":"") + "/> Continuous<br><br>";
  html += "<button>Save</button></form><p><a href='/'> Back</a></p>";
  html += htmlFooter();
  server.send(200, "text/html", html);
}

void handleSleepSave() {
  uint32_t sec = server.arg("sec").toInt();
  if (sec < 10) sec = 10;
  bool deep = (server.arg("m") == "d");
  
  sleepSec = sec;
  useDeepSleep = deep;
  saveConfigUInt("sleepSec", sec);
  saveConfigBool("deepSleep", deep);
  
  server.send(200, "text/plain", "Saved");
}

void handleMeasure() {
  server.send(200, "text/plain", "Measuring...");
  delay(100);
  measureAndPublish();
}

void handleRestart() {
  server.send(200, "text/plain", "Restarting...");
  delay(2000);
  ESP.restart();
}

void startHTTP() {
  if (httpStarted) return;
  server.on("/", handleRoot);
  server.on("/wifi", handleWiFi);
  server.on("/wifi/save", HTTP_POST, handleWiFiSave);
  server.on("/mqtt", handleMQTT);
  server.on("/mqtt/save", HTTP_POST, handleMQTTSave);
  server.on("/sleep", handleSleep);
  server.on("/sleep/save", HTTP_POST, handleSleepSave);
  server.on("/measure", handleMeasure);
  server.on("/restart", handleRestart);
  server.begin();
  httpStarted = true;
}

// =============================================================================
// STATE MACHINE
// =============================================================================

void transitionToState(SystemState newState) {
  if (newState == currentState) return;
  
  Serial.printf("\n>>> %s → %s <<<\n\n", getStateName(currentState), getStateName(newState));
  
  currentState = newState;
  stateStartTime = millis();
  setLEDPattern(newState);
  publishState(newState);
}

void handleStateInit() {
  errors.reset();
  Serial.println("=== INIT ===");
  
  if (!connectWiFi()) { transitionToState(STATE_ERROR_WIFI); startAP(); startHTTP(); return; }
  syncTime();
  if (!connectMQTT()) { transitionToState(STATE_ERROR_MQTT); startAP(); startHTTP(); return; }
  if (!initSD()) { transitionToState(STATE_ERROR_OTHER); startAP(); startHTTP(); return; }
  if (!initSensor()) { transitionToState(STATE_ERROR_OTHER); startAP(); startHTTP(); return; }
  
  Serial.println("=== ALL OK ===\n");
  stopAP();
  startHTTP();
  transitionToState(STATE_NORMAL);
  lastMeasure = millis() - MEASURE_INTERVAL_MS + 1000;
}

void handleStateNormal() {
  if (mqttConnected) mqtt.loop();
  checkSD();
  
  // Periodic state update
  if (millis() - lastStatePublish > STATE_PUBLISH_INTERVAL) {
    lastStatePublish = millis();
    publishState(currentState);
  }
  
  // Check connections
  if (WiFi.status() != WL_CONNECTED) {
    errors.wifi = true;
    transitionToState(STATE_ERROR_WIFI);
    startAP();
    return;
  }
  
  if (!mqtt.connected() && !connectMQTT()) {
    transitionToState(STATE_ERROR_MQTT);
    startAP();
    return;
  }
  
  // Measurement interval
  if (millis() - lastMeasure < MEASURE_INTERVAL_MS) return;
  lastMeasure = millis();
  
  buttonPressed = false;
  waitForFullMinute();
  
  if (buttonPressed && currentState == STATE_CONFIG_MANUAL) return;
  
  if (!measureAndPublish()) {
    if (errors.wifi) transitionToState(STATE_ERROR_WIFI);
    else if (errors.mqtt) transitionToState(STATE_ERROR_MQTT);
    else transitionToState(STATE_ERROR_OTHER);
    startAP();
    return;
  }
  
  if (!useDeepSleep) return;
  
  // Deep-Sleep preparation
  handleButton();
  if (buttonPressed && currentState == STATE_CONFIG_MANUAL) return;
  
  uint32_t sleepTime = calculateSleepTime();
  Serial.printf("💤 Sleep %us\n", sleepTime);
  
  StaticJsonDocument<128> doc;
  doc["state"] = "SLEEPING";
  doc["duration"] = sleepTime;
  char buf[128];
  serializeJson(doc, buf);
  mqtt.publish("smarthome/senderniklas/state", buf, true);
  delay(100);
  
  mqtt.publish("smarthome/senderniklas/status", "offline", true);
  delay(100);
  mqtt.disconnect();
  delay(100);
  WiFi.disconnect(true);
  delay(100);
  
  digitalWrite(LED_WIFI, LOW);
  digitalWrite(LED_ERROR, LOW);
  
  esp_sleep_enable_timer_wakeup(sleepTime * 1000000ULL);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_14, 0);
  esp_deep_sleep_start();
}

void handleStateConfigManual() {
  if (mqttConnected) mqtt.loop();
  if (millis() - stateStartTime > CONFIG_TIMEOUT_MS) {
    stopAP();
    transitionToState(STATE_NORMAL);
    lastMeasure = millis();
  }
}

void handleStateErrorWiFi() {
  if (millis() - stateStartTime > 30000) {
    if (connectWiFi()) transitionToState(STATE_INIT);
    else stateStartTime = millis();
  }
  if (millis() - stateStartTime > DEEP_SLEEP_TIMEOUT_MS) {
    transitionToState(STATE_DEEP_SLEEP_ONLY);
  }
}

void handleStateErrorMQTT() {
  if (!wifiConnected) { transitionToState(STATE_ERROR_WIFI); return; }
  if (millis() - stateStartTime > 30000) {
    if (connectMQTT()) transitionToState(STATE_INIT);
    else stateStartTime = millis();
  }
  if (millis() - stateStartTime > DEEP_SLEEP_TIMEOUT_MS) {
    transitionToState(STATE_DEEP_SLEEP_ONLY);
  }
}

void handleStateErrorOther() {
  if (millis() - stateStartTime > 15000) {
    bool fixed = true;
    if (errors.sd && !initSD()) fixed = false;
    if (errors.sensor && !initSensor()) fixed = false;
    if (fixed) { transitionToState(STATE_INIT); return; }
    stateStartTime = millis();
  }
  if (millis() - stateStartTime > DEEP_SLEEP_TIMEOUT_MS) {
    transitionToState(STATE_DEEP_SLEEP_ONLY);
  }
}

void handleStateDeepSleepOnly() {
  Serial.println("💀 Emergency sleep (reset to wake)");
  delay(100);
  mqtt.disconnect();
  WiFi.disconnect(true);
  stopAP();
  digitalWrite(LED_WIFI, LOW);
  digitalWrite(LED_ERROR, LOW);
  esp_sleep_enable_timer_wakeup(0xFFFFFFFFFFFFFFFFULL);
  esp_deep_sleep_start();
}

// =============================================================================
// MAIN
// =============================================================================

void setup() {
  Serial.begin(115200);
  delay(100);
  
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(LED_WIFI, OUTPUT);
  pinMode(LED_ERROR, OUTPUT);
  Wire.begin(I2C_SDA, I2C_SCL);
  ledcSetup(LED_PWM_CHANNEL, LED_PWM_FREQ, LED_PWM_RESOLUTION);
  
  Serial.println("\n\n╔════════════════════════════════════╗");
  Serial.println("║  ESP32 Environmental Logger v1.1   ║");
  Serial.println("╚════════════════════════════════════╝\n");
  
  // Check wakeup reason
  esp_sleep_wakeup_cause_t reason = esp_sleep_get_wakeup_cause();
  if (reason == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("🔘 Wakeup: Button");
    buttonPressed = true;
  } else if (reason == ESP_SLEEP_WAKEUP_TIMER) {
    Serial.println("⏰ Wakeup: Timer");
  } else {
    Serial.println("🔌 Wakeup: Power-on");
  }
  
  loadConfig();
  Serial.println("\n📋 Config:");
  Serial.printf("  WiFi:  %s\n", wifiSsid.c_str());
  Serial.printf("  MQTT:  %s:%d\n", mqttHost.c_str(), mqttPort);
  Serial.printf("  Sleep: %us (%s)\n\n", sleepSec, useDeepSleep ? "enabled" : "disabled");
  
  currentState = STATE_INIT;
  stateStartTime = millis();
  setLEDPattern(STATE_INIT);
}

void loop() {
  handleButton();
  updateLEDs();
  
  // Button wakeup handling
  if (buttonPressed && currentState == STATE_INIT) {
    static bool done = false;
    if (!done && currentState == STATE_NORMAL) {
      done = true;
      transitionToState(STATE_CONFIG_MANUAL);
      startAP();
      startHTTP();
      buttonPressed = false;
    }
  }
  
  if (httpStarted) server.handleClient();
  
  // State machine dispatch
  switch (currentState) {
    case STATE_INIT:            handleStateInit(); break;
    case STATE_NORMAL:          handleStateNormal(); break;
    case STATE_CONFIG_MANUAL:   handleStateConfigManual(); break;
    case STATE_ERROR_WIFI:      handleStateErrorWiFi(); break;
    case STATE_ERROR_MQTT:      handleStateErrorMQTT(); break;
    case STATE_ERROR_OTHER:     handleStateErrorOther(); break;
    case STATE_DEEP_SLEEP_ONLY: handleStateDeepSleepOnly(); break;
  }
  
  delay(10);
}