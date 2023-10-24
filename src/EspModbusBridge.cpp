#include <TelnetStream.h>
#include "EspModbusBridge.h"

#define MODBUS_TCP_PORT (502)

#define RTU_TIMEOUT_MS (500)
#define QUEUE_WDT_TIMEOUT (5000)
// This defines the size of the request queue
#define MAX_CONCURRENT_REQUESTS (4)

ModbusBridge::ModbusBridge(Stream& logStream) 
:log(logStream), requests(MAX_CONCURRENT_REQUESTS) { }

void ModbusBridge::begin(Stream& rtuStream, int16_t txEnablePin, ModbusRTUTxEnableMode txEnableMode) {
    tcp.server(MODBUS_TCP_PORT);
    tcp.onRaw([this](uint8_t* data, uint8_t len, void* src) -> Modbus::ResultCode { 
      return onTcpRaw(data, len, static_cast<Modbus::frame_arg_t*>(src));
    });
    tcp.onConnect([this](IPAddress ip) -> bool { 
      return onTcpConnected(ip);
    });
    tcp.onDisconnect([this](IPAddress ip) -> bool {
      return onTcpDisconnected(ip);
    });

    rtu.begin(&rtuStream, txEnablePin, txEnableMode);

    rtu.master();
    rtu.onRaw([this](uint8_t* data, uint8_t len, void* src) -> Modbus::ResultCode {
      return onRtuRaw(data, len, static_cast<Modbus::frame_arg_t*>(src));
    }, true);

    // Sofar doesn't follow the modbus spec, and it is splitting messages with more than 3.5 * space time sometimes
    // ((1 / 9600) * 11) * 3.5 = 4ms
    // Use 8ms instead
    rtu.setInterFrameTime(8000);
}

void ModbusBridge::task() {
    rtu.task();
    tcp.task();

    if (!requests.isEmpty()) {
        if (beginQueueActivityTs == 0) {
        beginQueueActivityTs = millis();
        } else if (millis() - beginQueueActivityTs > QUEUE_WDT_TIMEOUT) {
        // Reset MCU
        log.printf("Queue Watchdog: reset\n");
        esp_restart();
        }
    } else if (requests.isEmpty()) {
        beginQueueActivityTs = 0;
    }

    if (!requests.isEmpty() && !requests.inProgress()) {
        dequeueReq();
    }
    if (requests.inProgress() && millis() - requests.getTopTimestamp() > RTU_TIMEOUT_MS) {
        timeoutRtu();
    }
}

void ModbusBridge::sendErr(const ModbusBridge::PendingRequest& req, Modbus::ResultCode err) {
    log.printf("err, tcpTransId: %d, err: %d\n", req.tcpTransId, err);
    tcp.setTransactionId(req.tcpTransId); 
    if (!tcp.errorResponse(IPAddress(req.tcpIpaddr), static_cast<Modbus::FunctionCode>(req.data[0]), err, req.rtuNodeId)) {
      log.printf("TCP errResp failed\n");
    }
}

// Callback that receives raw TCP requests 
Modbus::ResultCode ModbusBridge::onTcpRaw(uint8_t* data, uint8_t len, Modbus::frame_arg_t* frameArg) {
  PendingRequest req;
  req.rtuNodeId = frameArg->slaveId;
  req.tcpTransId = frameArg->transactionId;
  req.tcpIpaddr = frameArg->ipaddr;
  memcpy(req.data, data, len);
  req.dataLen = len;

  if (requests.isFull()) {
    sendErr(req, Modbus::EX_PATH_UNAVAILABLE);
    log.printf("TCP nodeId: %d, Fn: %02X, QUEUE_FULL ERR\n: ", req.rtuNodeId, static_cast<Modbus::FunctionCode>(data[0]));
  } else {
    requests.push(req);
    log.printf("TCP nodeId: %d, Fn: %02X, len: %d\n: ", req.rtuNodeId, static_cast<Modbus::FunctionCode>(data[0]), len);
  }


  // Stop other processing
  return Modbus::EX_SUCCESS; 
}

void ModbusBridge::dequeueReq() {
  const PendingRequest& req = requests.peek();

  // Must save transaction ans node it for response processing
  if (!rtu.rawRequest(req.rtuNodeId, req.data, req.dataLen)) {
    // rawRequest returns 0 is unable to send data for some reason
    // do nothing and wait
    log.printf("RTU err: tcpTransId: %d\n", req.tcpTransId);
  } else {
    requests.setTopInProgress();
    // Sent. Now wait for response
    log.printf("rtuNodeId: %d, tcpTransId: %d\n", req.rtuNodeId, req.tcpTransId);
  }
}

bool ModbusBridge::onTcpConnected(IPAddress ip) {
  log.print("TCP connected from: ");
  log.print(ip);
  log.print("\n");
  return true;
}

bool ModbusBridge::onTcpDisconnected(IPAddress ip) {
  log.print("TCP disconnected from: ");
  log.print(ip);
  log.print("\n");
  return true;
}

/**
 * Fix frame errors of the first byte due to bus arbitration/drive switch 
 */
void ModbusBridge::tryFixFrame(const PendingRequest req, Modbus::frame_arg_t* frameArg, uint8_t*& data, uint8_t& len) {
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
    log.printf("Recovered 0x90 error\n");
  } else if (data[0] == req.rtuNodeId && data[1] == req.data[0]) {
    // 1-shift is common, the node Id entered in the frame shifting everything up
    len--;
    data++;
    frameArg->validFrame = true;
    frameArg->slaveId = req.rtuNodeId;
    log.printf("Recovered 1-byte shifted frame\n");
  } else {
      log.printf("RTU: Invalid frame: ");
      uint8_t i;
      for (i = 0; i < len + 2 && i < 64; i++) {
        log.printf("<%02x>", data[i]);
      }
      if (i >= 64) {
        log.printf("...");
      }
      log.printf("\n");
  }
}

// Callback that receives raw responses from RTU
Modbus::ResultCode ModbusBridge::onRtuRaw(uint8_t* data, uint8_t len, Modbus::frame_arg_t* frameArg) {
  auto funCode = static_cast<Modbus::FunctionCode>(data[0]);

  log.printf("RTU: Fn: %02X, len: %d, nodeId: %d, to_server: %d, validFrame: %d\n", funCode, len, frameArg->slaveId, frameArg->to_server, frameArg->validFrame);
  if (!requests.inProgress()) {
    log.printf("RTU: ignored, not in progress, rtuNodeId: %d\n", frameArg->slaveId);
  } else if (frameArg->to_server) {
    log.printf("RTU: ignored, not a response, rtuNodeId: %d\n", frameArg->slaveId);
  } else {
    const auto& req = requests.stopTopInProgress();

    // Check if transaction id is match
    if (!frameArg->validFrame || req.rtuNodeId != frameArg->slaveId) {
      tryFixFrame(req, frameArg, data, len);
    }

    if (frameArg->validFrame && req.rtuNodeId == frameArg->slaveId) {
      tcp.setTransactionId(req.tcpTransId);
      // Put back the rtuNodeId otherwise it will respond with the master TCP node address
      if (!tcp.rawResponse(IPAddress(req.tcpIpaddr), data, len, req.rtuNodeId)) {
        log.printf("TCP rawResponse failed\n");
      }
    } else {
      // Closes the request
      sendErr(req, Modbus::EX_DEVICE_FAILED_TO_RESPOND);
    }
  }
  return Modbus::EX_SUCCESS; // Stop other processing
}

void ModbusBridge::timeoutRtu() {
  const auto& req = requests.stopTopInProgress();
  log.printf("RTU: timeout, tcpTransId: %d\n", req.tcpTransId);
  sendErr(req, Modbus::EX_DEVICE_FAILED_TO_RESPOND);
}

TelnetModbusBridge::TelnetModbusBridge()
:ModbusBridge(TelnetStream) {
}

void TelnetModbusBridge::begin(Stream& rtuStream, int16_t txEnablePin, ModbusRTUTxEnableMode txEnableMode) {
    TelnetStream.begin();
    ModbusBridge::begin(rtuStream, txEnablePin, txEnableMode);
}


void TelnetModbusBridge::task() {
  ModbusBridge::task();
  // Clear RX buffer
  while (log.available() > 0) {
      log.read();
  }
}