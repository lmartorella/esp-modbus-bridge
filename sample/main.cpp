#if defined(ESP8266)
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif

#include <HardwareSerial.h>
#include <EspModbusBridge.h>
#include "wifi_ssid.h"

#if defined(ESP8266)
// For ESP8266, use the main serial port, always exposed also in minimal ESP-01 module
static HardwareSerial& _rtuSerial = Serial;
#else
// Use port 1
static HardwareSerial _rtuSerial(1);
#endif

#if defined(ESP8266)
#define RS485_OUTPUT_ENABLE_PIN 0
#else
#define RS485_OUTPUT_ENABLE_PIN 32
#endif

static TelnetModbusBridge bridge;

void setup() {
    // Init serial port, default pins, to drive the RS-485 line
    _rtuSerial.begin(9600, SERIAL_8N1);

    // Start WiFi
    WiFi.setHostname(WIFI_HOSTNAME);
    WiFi.begin(WIFI_SSID, WIFI_PASSPHRASE);

    // Init the bridge, selecting the right GPIO for the RS-485 driver enable pin, with a high-level logic (matches the MAX485/MAX4485)
    bridge.begin(_rtuSerial, RS485_OUTPUT_ENABLE_PIN, TxEnableHigh);
}

void loop() {
    bridge.task();
    yield();
}
