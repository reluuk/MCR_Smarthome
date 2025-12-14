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
