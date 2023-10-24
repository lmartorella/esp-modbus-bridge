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

#include <TelnetStream.h>
#include <HardwareSerial.h>
#include <EspModbusBridge.h>
#include "wifi_ssid.h"

static TelnetStreamClass& _log =  TelnetStream; // TX only

#if defined(ESP8266)
static HardwareSerial& _rtuSerial = Serial;
#else
static HardwareSerial _rtuSerial(1);
#endif

#if defined(ESP32)
static WiFiUDP udp;
static MDNS mdns(udp);
#endif

static ModbusBridge bridge(_log, _rtuSerial);

void setup() {
#if defined(ESP8266)
    _rtuSerial.begin(9600, SERIAL_8N1);
#else
    _rtuSerial.begin(9600, SERIAL_8N1, 35, 33); // 35 is input only

    // Keep Serial1 healthy for debugging
    Serial.begin(115200, SERIAL_8N1);
    Serial.end();
    Serial.begin(115200, SERIAL_8N1);
#endif

    WiFi.setHostname(WIFI_HOSTNAME);
    WiFi.begin(WIFI_SSID, WIFI_PASSPHRASE);

    TelnetStream.begin();

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

    // Clear RX buffer
    while (_log.available() > 0) {
        _log.read();
    }

    bridge.task();
    yield();
}