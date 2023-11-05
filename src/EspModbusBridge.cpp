#include <TelnetStream.h>
#include "EspModbusBridge.h"

const int MODBUS_TCP_PORT = 502;

// Hard-coded timeout if the RTU node doesn't respond. This triggers a EX_DEVICE_FAILED_TO_RESPOND modbus error
// It must be greater than MODBUSRTU_TIMEOUT, since the modbus library will cleanup the state after this time.
const int RTU_TIMEOUT_MS = (MODBUSRTU_TIMEOUT + 50);

// If the queue is not getting emptied in this time, reset the MCU
const int QUEUE_WDT_TIMEOUT = 5000;
// This defines the size of the request queue
const int MAX_CONCURRENT_REQUESTS = 4;

ModbusBridge::ModbusBridge(Stream& logStream) 
:requests(MAX_CONCURRENT_REQUESTS), log(logStream) { }

void ModbusBridge::begin(Stream& rtuStream, int16_t txEnablePin, ModbusRTUTxEnableMode txEnableMode) {
    tcp.server(MODBUS_TCP_PORT);
    tcp.onRaw([this](uint8_t* data, uint8_t len, void* src) -> Modbus::ResultCode { 
        return onTcpRaw(data, len, static_cast<Modbus::frame_arg_t*>(src));
    });
    tcp.onConnect([this](IPAddress ip) -> bool { 
        return onTcpConnected(ip);
    });
    tcp.onDisconnect([this](IPAddress ip) -> bool {
        // modbus lib always sends IPADDR_NONE to disconnect
        // TODO: fix this
        return onTcpDisconnected();
    });

    rtu.begin(&rtuStream, txEnablePin, txEnableMode);

    rtu.master();
    rtu.onRaw([this](uint8_t* data, uint8_t len, void* src) -> Modbus::ResultCode {
        return onRtuRaw(data, len, static_cast<Modbus::frame_arg_t*>(src));
    }, true);
}

void ModbusBridge::setInterFrameTime(int us) {
    rtu.setInterFrameTime(us);
}

void ModbusBridge::task() {
    rtu.task();
    tcp.task();

    if (!requests.isEmpty()) {
        if (beginQueueActivityTs == 0) {
            beginQueueActivityTs = millis();
        } else if (millis() - beginQueueActivityTs > QUEUE_WDT_TIMEOUT) {
            // Reset MCU
            log.printf("ERR: queue watchdog: reset\n");
#ifdef ESP32
            esp_restart();
#else
            ESP.restart();
#endif
        }
    } else if (requests.isEmpty()) {
        beginQueueActivityTs = 0;
    }

    auto ts = requests.getHeadTimestamp();
    if (!requests.isEmpty() && !requests.inProgress()) {
        // Process head
        dequeueReq();
    }
    else if (requests.inProgress() && millis() - ts > RTU_TIMEOUT_MS) {
        timeoutRtu();
    }
}

void ModbusBridge::sendErr(const ModbusBridge::PendingRequest& req, Modbus::ResultCode err) {
    log.printf("RESP-ERR: code: %d, tcpTransId: %d\n", err, req.tcpTransId);
    tcp.setTransactionId(req.tcpTransId); 
    if (!tcp.errorResponse(IPAddress(req.tcpIpaddr), static_cast<Modbus::FunctionCode>(req.data[0]), err, req.rtuNodeId)) {
        log.printf("TCP: errResp failed\n");
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
    } else {
        requests.push(req);
        log.printf("REQ: nodeId: %d, fun: %02X, len: %d, tcpTransId: %d\n", req.rtuNodeId, static_cast<Modbus::FunctionCode>(data[0]), len, req.tcpTransId);
    }

    // Stop other processing
    return Modbus::EX_SUCCESS; 
}

void ModbusBridge::dequeueReq() {
    const PendingRequest& req = requests.peek();

    // Must save transaction ans node it for response processing
    if (!rtu.rawRequest(req.rtuNodeId, req.data, req.dataLen)) {
        // rawRequest returns 0 is unable to send data for some reason
        log.printf("RTU: rawRequest failed: tcpTransId: %d\n", req.tcpTransId);
    } else {
        requests.setHeadInProgress();
        // Sent. Now wait for response
        log.printf("REQ: on-the-wire rtuNodeId: %d, tcpTransId: %d\n", req.rtuNodeId, req.tcpTransId);
    }
}

bool ModbusBridge::onTcpConnected(IPAddress ip) {
    log.print("TCP connected from: ");
    log.print(ip);
    log.print("\n");
    return true;
}

bool ModbusBridge::onTcpDisconnected() {
    log.print("TCP disconnected\n");
    return true;
}

// Callback that receives raw responses from RTU
Modbus::ResultCode ModbusBridge::onRtuRaw(uint8_t* data, uint8_t len, Modbus::frame_arg_t* frameArg) {
    auto funCode = static_cast<Modbus::FunctionCode>(data[0]);

    if (!requests.inProgress()) {
        log.printf("RTU: ignored, not in progress, rtuNodeId: %d\n", frameArg->slaveId);
    } else if (frameArg->to_server) {
        log.printf("RTU: ignored, not a response, rtuNodeId: %d\n", frameArg->slaveId);
    } else {
        log.printf("RESP: fn: %02X, len: %d, nodeId: %d, validFrame: %d\n", funCode, len, frameArg->slaveId, frameArg->validFrame);
        const auto& req = requests.dequeue();

        // Check if transaction id is match
        if (!frameArg->validFrame || req.rtuNodeId != frameArg->slaveId) {
            tryFixFrame(req.rtuNodeId, req.data[0], frameArg, data, len);
        }

        if (frameArg->validFrame && req.rtuNodeId == frameArg->slaveId) {
            tcp.setTransactionId(req.tcpTransId);
            // Put back the rtuNodeId otherwise it will respond with the master TCP node address
            if (!tcp.rawResponse(IPAddress(req.tcpIpaddr), data, len, req.rtuNodeId)) {
                log.printf("TCP: rawResponse failed\n");
            }
        } else {
            // Closes the request
            sendErr(req, Modbus::EX_DEVICE_FAILED_TO_RESPOND);
        }
    }
    return Modbus::EX_SUCCESS; // Stop other processing
}

void ModbusBridge::timeoutRtu() {
    const auto& req = requests.dequeue();
    log.printf("REQ: timeout, tcpTransId: %d\n", req.tcpTransId);
    sendErr(req, Modbus::EX_DEVICE_FAILED_TO_RESPOND);
}

TelnetModbusBridge::TelnetModbusBridge()
    :ModbusBridge(TelnetStream) { }

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