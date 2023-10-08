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

#include <ModbusRTU.h>
#include <ModbusTCP.h>
#include <TelnetStream.h>
#include <HardwareSerial.h>
#include "wifi_ssid.h"

static ModbusRTU rtu;
static ModbusTCP tcp;
static TelnetStreamClass& _log = TelnetStream; // TX only

#if defined(ESP8266)
static HardwareSerial& _rtuSerial = Serial;
#else
static HardwareSerial _rtuSerial(1);
#endif

#if defined(ESP32)
static WiFiUDP udp;
static MDNS mdns(udp);
#endif

static const int MODBUS_TCP_PORT = 502;
static uint16_t rtuNodeId = 0;
static uint16_t tcpTransId = 0;
static uint32_t tcpIpaddr = 0;
static Modbus::FunctionCode tcpFunCode;

// Callback that receives raw TCP requests 
static Modbus::ResultCode cbTcpRaw(uint8_t* data, uint8_t len, void* custom) {
  auto src = (Modbus::frame_arg_t*) custom;
  auto nodeId = src->slaveId;
  tcpFunCode = static_cast<Modbus::FunctionCode>(data[0]);

  _log.printf("TCP nodeId: %d, Fn: %02X, len: %d\n: ", nodeId, tcpFunCode, len);
  tcpTransId = src->transactionId;
  tcpIpaddr = src->ipaddr;
  rtuNodeId = nodeId;

  // Must save transaction ans node it for response processing
  if (!rtu.rawRequest(nodeId, data, len)) {
    // rawRequest returns 0 is unable to send data for some reason
    _log.printf("err, tcpTransId: %d\n", tcpTransId);
    tcp.setTransactionId(tcpTransId); 
    if (!tcp.errorResponse(IPAddress(tcpIpaddr), tcpFunCode, Modbus::EX_DEVICE_FAILED_TO_RESPOND, rtuNodeId)) {
      _log.printf("TCP errorResponse failed\n");
    }
  } else {
    _log.printf("rtuNodeId: %d, tcpTransId: %d\n", rtuNodeId, tcpTransId);
  }
  
  // Stop other processing
  return Modbus::EX_SUCCESS; 
}

static bool onTcpConnected(IPAddress ip) {
  _log.print("TCP connected from: ");
  _log.print(ip);
  _log.print("\n");
  return true;
}

static bool onTcpDisconnected(IPAddress ip) {
  _log.print("TCP disconnected from: ");
  _log.print(ip);
  _log.print("\n");
  return true;
}

/**
 * Fix frame errors of the first byte due to bus arbitration/drive switch 
 */
static void tryFixFrame(Modbus::frame_arg_t* frameArg, uint8_t*& data, uint8_t& len) {
  if (len == 3 && data[1] == 0x90) {
    // Shift 1
    len--;
    data++;
  }
  if (len == 2 && data[0] == 0x90) {
    // Fix Sofar error
    data[0] = 0x83;
    frameArg->validFrame = true;
    frameArg->slaveId = rtuNodeId;
    _log.printf("Recovered 0x90 error\n");
  } else if (data[0] == rtuNodeId && data[1] == tcpFunCode) {
    // 1-shift is common, the node Id entered in the frame shifting everything up
    len--;
    data++;
    frameArg->validFrame = true;
    frameArg->slaveId = rtuNodeId;
    _log.printf("Recovered 1-byte shifted frame\n");
  } else {
      _log.printf("RTU: Invalid frame: ");
      uint8_t i;
      for (i = 0; i < len + 2 && i < 64; i++) {
        _log.printf("<%02x>", data[i]);
      }
      if (i >= 64) {
        _log.printf("...");
      }
      _log.printf("\n");
  }
}

// Callback that receives raw responses
static Modbus::ResultCode cbRtuRaw(uint8_t* data, uint8_t len, void* custom) {
  auto frameArg = (Modbus::frame_arg_t*) custom;
  auto funCode = static_cast<Modbus::FunctionCode>(data[0]);

  _log.printf("RTU: Fn: %02X, len: %d, nodeId: %d, to_server: %d, validFrame: %d\n", funCode, len, frameArg->slaveId, frameArg->to_server, frameArg->validFrame);
  if (!frameArg->to_server) { 
    // Check if transaction id is match
    if (!frameArg->validFrame || rtuNodeId != frameArg->slaveId) {
      tryFixFrame(frameArg, data, len);
    } else {
      // uint8_t i;
      // for (i = 0; i < len + 2 && i < 64; i++) {
      //   _log.printf("<%02x>", data[i]);
      // }
      // if (i >= 64) {
      //   _log.printf("...");
      // }
      // _log.printf("\n");
    }

    if (frameArg->validFrame && rtuNodeId == frameArg->slaveId) {
      tcp.setTransactionId(tcpTransId);
      // Put back the rtuNodeId otherwise it will respond with the master TCP node address
      if (!tcp.rawResponse(IPAddress(tcpIpaddr), data, len, rtuNodeId)) {
        _log.printf("TCP rawResponse failed\n");
      }
    }
    rtuNodeId = 0;
    tcpTransId = 0;
    tcpIpaddr = 0;
  } else {
    _log.printf("RTU: ignored, not in progress, rtuNodeId: %d\n", rtuNodeId);
  }
  return Modbus::EX_SUCCESS; // Stop other processing
}

void setup() {
#if defined(ESP8266)
  _rtuSerial.begin(9600, SERIAL_8N1);
#else
  _rtuSerial.begin(9600, SERIAL_8N1, 35, 33); // 35 is input only
#endif

  WiFi.setHostname(WIFI_HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASSPHRASE);
  
  tcp.server(MODBUS_TCP_PORT);
  tcp.onRaw(cbTcpRaw);
  tcp.onConnect(onTcpConnected);
  tcp.onDisconnect(onTcpDisconnected);
  
#if defined(ESP8266)
  rtu.begin(&_rtuSerial, 0, TxEnableHigh);
#else
  rtu.begin(&_rtuSerial, 32, TxEnableHigh);
#endif
  rtu.master();
  rtu.onRaw(cbRtuRaw, true);

  TelnetStream.begin();

#if defined(ESP32)
  // Initialize the mDNS library. You can now reach or ping this
  // Arduino via the host name "arduino.local", provided that your operating
  // system is mDNS/Bonjour-enabled (such as MacOS X).
  // Always call this before any other method!
  mdns.begin(WiFi.localIP(), "esp32");
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

  rtu.task();
  tcp.task();
  yield();
}