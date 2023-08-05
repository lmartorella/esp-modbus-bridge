#if defined(ESP8266)
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif

#include <WiFiUdp.h>
#include <ArduinoMDNS.h>
#include <ModbusRTU.h>
#include <ModbusTCP.h>
#include <TelnetStream.h>
#include <HardwareSerial.h>
#include "wifi_ssid.h"

static ModbusRTU rtu;
static ModbusTCP tcp;
static TelnetStreamClass& _log = TelnetStream; // TX only
static HardwareSerial _rtuSerial(1);

static WiFiUDP udp;
static MDNS mdns(udp);

static const int MODBUS_TCP_PORT = 502;
static uint16_t rtuNodeId = 0;
static uint16_t tcpTransId = 0;
static uint32_t tcpIpaddr = 0;

// Callback that receives raw TCP requests 
static Modbus::ResultCode cbTcpRaw(uint8_t* data, uint8_t len, void* custom) {
  auto src = (Modbus::frame_arg_t*) custom;
  auto nodeId = src->slaveId;
  auto funCode = static_cast<Modbus::FunctionCode>(data[0]);

  _log.printf("TCP nodeId: %d, Fn: %02X, len: %d\n: ", nodeId, funCode, len);

  // Must save transaction ans node it for response processing
  auto ret = rtu.rawRequest(nodeId, data, len);
  if (!ret) {
    // rawRequest returns 0 is unable to send data for some reason
    _log.printf("err\n");
    tcp.setTransactionId(src->transactionId); 
    tcp.errorResponse(IPAddress(src->ipaddr), funCode, Modbus::EX_DEVICE_FAILED_TO_RESPOND);
  } else {
    tcpTransId = src->transactionId;
    tcpIpaddr = src->ipaddr;
    rtuNodeId = nodeId;
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

// Callback that receives raw responses
Modbus::ResultCode cbRtuRaw(uint8_t* data, uint8_t len, void* custom) {
  auto src = (Modbus::frame_arg_t*) custom;
  auto nodeId = src->slaveId;
  auto funCode = static_cast<Modbus::FunctionCode>(data[0]);

  _log.printf("RTU: Fn: %02X, len: %d, nodeId: %d, to_server: %d\n", funCode, len, nodeId, src->to_server);
  if (!src->to_server && rtuNodeId == nodeId) { // Check if transaction id is match
    tcp.setTransactionId(tcpTransId); 
    if (!tcp.rawResponse(IPAddress(tcpIpaddr), data, len)) {
      _log.printf("TCP rawResponse failed\n");
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
  _rtuSerial.begin(9600, SERIAL_8N1, 35, 33); // 35 is input only
  
  WiFi.setHostname(WIFI_HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASSPHRASE);
  
  tcp.server(MODBUS_TCP_PORT);
  tcp.onRaw(cbTcpRaw);
  tcp.onConnect(onTcpConnected);
  tcp.onDisconnect(onTcpDisconnected);
  
  rtu.begin(&_rtuSerial, 32, TxEnableHigh);
  rtu.master();
  rtu.onRaw(cbRtuRaw); 

  TelnetStream.begin();

  // Initialize the mDNS library. You can now reach or ping this
  // Arduino via the host name "arduino.local", provided that your operating
  // system is mDNS/Bonjour-enabled (such as MacOS X).
  // Always call this before any other method!
  mdns.begin(WiFi.localIP(), "esp32");
}

void loop() {
  // This actually runs the mDNS module. YOU HAVE TO CALL THIS PERIODICALLY,
  // OR NOTHING WILL WORK! Preferably, call it once per loop().
  mdns.run();

  // Clear RX buffer
  while (_log.available() > 0) {
    _log.read();
  }

  rtu.task();
  tcp.task();
  yield();
}