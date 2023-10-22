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
#include "queue.h"
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
#define MODBUS_PDU_MAX_SIZE (253)
#define RTU_TIMEOUT_MS (500)

struct PendingRequest {
  uint8_t rtuNodeId;
  uint16_t tcpTransId;
  uint32_t tcpIpaddr;
  
  uint8_t data[MODBUS_PDU_MAX_SIZE];
  uint8_t dataLen;
};

// Max 4 concurrent connections
static FifoQueue<PendingRequest> requests(4);

static void sendErr(const PendingRequest& req, Modbus::ResultCode err) {
    _log.printf("err, tcpTransId: %d\n", req.tcpTransId);
    tcp.setTransactionId(req.tcpTransId); 
    if (!tcp.errorResponse(IPAddress(req.tcpIpaddr), static_cast<Modbus::FunctionCode>(req.data[0]), err, req.rtuNodeId)) {
      _log.printf("TCP errorResponse failed\n");
    }
}

// Callback that receives raw TCP requests 
static Modbus::ResultCode cbTcpRaw(uint8_t* data, uint8_t len, void* custom) {
  auto src = (Modbus::frame_arg_t*) custom;
  PendingRequest req;
  req.rtuNodeId = src->slaveId;
  req.tcpTransId = src->transactionId;
  req.tcpIpaddr = src->ipaddr;
  memcpy(req.data, data, len);
  req.dataLen = len;

  if (requests.isFull()) {
    sendErr(req, Modbus::EX_PATH_UNAVAILABLE);
    _log.printf("TCP nodeId: %d, Fn: %02X, QUEUE_FULL ERR\n: ", req.rtuNodeId, static_cast<Modbus::FunctionCode>(data[0]));
  } else {
    requests.push(req);
    _log.printf("TCP nodeId: %d, Fn: %02X, len: %d\n: ", req.rtuNodeId, static_cast<Modbus::FunctionCode>(data[0]), len);
  }


  // Stop other processing
  return Modbus::EX_SUCCESS; 
}

static void dequeueReq() {
  const PendingRequest& req = requests.peek();

  // Must save transaction ans node it for response processing
  if (!rtu.rawRequest(req.rtuNodeId, req.data, req.dataLen)) {
    // rawRequest returns 0 is unable to send data for some reason
    // do nothing and wait
    _log.printf("RTU err: tcpTransId: %d\n", req.tcpTransId);
  } else {
    requests.setTopInProgress();
    // Sent. Now wait for response
    _log.printf("rtuNodeId: %d, tcpTransId: %d\n", req.rtuNodeId, req.tcpTransId);
  }
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
static void tryFixFrame(const PendingRequest req, Modbus::frame_arg_t* frameArg, uint8_t*& data, uint8_t& len) {
  if (len == 3 && data[1] == 0x90) {
    // Shift 1
    len--;
    data++;
  }
  if (len == 2 && data[0] == 0x90) {
    // Fix Sofar error
    data[0] = 0x83;
    frameArg->validFrame = true;
    frameArg->slaveId = req.rtuNodeId;
    _log.printf("Recovered 0x90 error\n");
  } else if (data[0] == req.rtuNodeId && data[1] == req.data[0]) {
    // 1-shift is common, the node Id entered in the frame shifting everything up
    len--;
    data++;
    frameArg->validFrame = true;
    frameArg->slaveId = req.rtuNodeId;
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

// Callback that receives raw responses from RTU
static Modbus::ResultCode cbRtuRaw(uint8_t* data, uint8_t len, void* custom) {
  auto frameArg = (Modbus::frame_arg_t*) custom;
  auto funCode = static_cast<Modbus::FunctionCode>(data[0]);

  _log.printf("RTU: Fn: %02X, len: %d, nodeId: %d, to_server: %d, validFrame: %d\n", funCode, len, frameArg->slaveId, frameArg->to_server, frameArg->validFrame);
  if (!frameArg->to_server) {
    const auto& req = requests.stopTopInProgress();

    // Check if transaction id is match
    if (!frameArg->validFrame || req.rtuNodeId != frameArg->slaveId) {
      tryFixFrame(req, frameArg, data, len);
    }

    if (frameArg->validFrame && req.rtuNodeId == frameArg->slaveId) {
      tcp.setTransactionId(req.tcpTransId);
      // Put back the rtuNodeId otherwise it will respond with the master TCP node address
      if (!tcp.rawResponse(IPAddress(req.tcpIpaddr), data, len, req.rtuNodeId)) {
        _log.printf("TCP rawResponse failed\n");
      }
    }
  } else {
    _log.printf("RTU: ignored, not in progress, rtuNodeId: %d\n", frameArg->slaveId);
  }
  return Modbus::EX_SUCCESS; // Stop other processing
}

static void timeoutRtu() {
  const auto& req = requests.stopTopInProgress();
  _log.printf("RTU: timeout, tcpTransId: %d\n", req.tcpTransId);
  sendErr(req, Modbus::EX_DEVICE_FAILED_TO_RESPOND);
}

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
  // Sofar doesn't follow the modbus spec, and it is splitting messages with more than 3.5 * space time sometimes
  // ((1 / 9600) * 11) * 3.5 = 4ms
  // Use 8ms instead
  rtu.setInterFrameTime(8000);

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

  rtu.task();
  tcp.task();

  if (!requests.isEmpty() && !requests.inProgress()) {
    dequeueReq();
  }
  if (requests.inProgress() && millis() - requests.getTopTimestamp() > RTU_TIMEOUT_MS) {
    timeoutRtu();
  }
  yield();
}