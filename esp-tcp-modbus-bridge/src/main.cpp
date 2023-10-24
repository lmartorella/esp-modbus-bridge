#if defined(ESP8266)
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif

#if defined(ESP32)
// Not compatible with Arduino UDP of esp8266 core yet
#include <WiFiUdp.h>
#include <ArduinoMDNS.h>
#endif

#include <HardwareSerial.h>
#include <EspModbusBridge.h>
#include "wifi_ssid.h"

#if defined(ESP8266)
static HardwareSerial& _rtuSerial = Serial;
#else
static HardwareSerial _rtuSerial(1);
#endif

#if defined(ESP32)
static WiFiUDP udp;
static MDNS mdns(udp);
#endif

static TelnetModbusBridge bridge;

void setup() {
#if defined(ESP32)
    // Keep Serial1 healthy for debugging
    Serial.begin(115200, SERIAL_8N1);
    Serial.end();
    Serial.begin(115200, SERIAL_8N1);
#endif

#if defined(ESP8266)
    _rtuSerial.begin(9600, SERIAL_8N1);
#else
    _rtuSerial.begin(9600, SERIAL_8N1, 35, 33); // 35 is input only
#endif

#if defined(ESP8266)
    bridge.begin(_rtuSerial, 0, TxEnableHigh);
#else
    bridge.begin(_rtuSerial, 32, TxEnableHigh);
#endif

    WiFi.setHostname(WIFI_HOSTNAME);
    WiFi.begin(WIFI_SSID, WIFI_PASSPHRASE);

#if defined(ESP32)
    // Initialize the mDNS library. You can now reach or ping this
    // Arduino via the host name "arduino.local", provided that your operating
    // system is mDNS/Bonjour-enabled (such as MacOS X).
    // Always call this before any other method!
    mdns.begin(WiFi.localIP(), WIFI_HOSTNAME);
#endif
}

void loop() {
#if defined(ESP32)
    // This actually runs the mDNS module. YOU HAVE TO CALL THIS PERIODICALLY,
    // OR NOTHING WILL WORK! Preferably, call it once per loop().
    mdns.run();
#endif

    bridge.task();
    yield();
}