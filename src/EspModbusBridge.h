#pragma once

#include <ModbusRTU.h>
#include <ModbusTCP.h>
#include <TelnetStream.h>
#include "queue.h"

#define MODBUS_PDU_MAX_SIZE (253)

/**
 * Bridge between TCP (WiFi) and RTU (RS485) Modbus.
 * Written for ESP8266/ESP32
 */
class ModbusBridge {
    ModbusRTU rtu;
    ModbusTCP tcp;
    // Try to fix deadlock in modbus, that stops to be able to dequeue requests
    // Timestamp of the first object entered in queue, reset when the queue empties
    unsigned long beginQueueActivityTs = 0;

    struct PendingRequest {
        uint8_t rtuNodeId;
        uint16_t tcpTransId;
        uint32_t tcpIpaddr;
        
        uint8_t data[MODBUS_PDU_MAX_SIZE];
        uint8_t dataLen;
    };

    FifoQueue<PendingRequest> requests;

    void sendErr(const PendingRequest& req, Modbus::ResultCode err);
    void dequeueReq();

    // Callback that receives raw TCP requests 
    Modbus::ResultCode onTcpRaw(uint8_t* data, uint8_t len, Modbus::frame_arg_t* frameArg);
    // Callback that receives raw responses from RTU
    Modbus::ResultCode onRtuRaw(uint8_t* data, uint8_t len, Modbus::frame_arg_t* frameArg);

    bool onTcpConnected(IPAddress ip);
    bool onTcpDisconnected(IPAddress ip);

    void timeoutRtu();

protected:
    Stream& log;
    /**
     * Fix frame errors of the first byte due to bus arbitration/drive switch 
     */
    virtual void tryFixFrame(uint8_t rtuNodeId, uint8_t requestFunction, Modbus::frame_arg_t* frameArg, uint8_t*& data, uint8_t& len) { }

public:
    ModbusBridge(Stream& logStream);
    void begin(Stream& rtuStream, int16_t txEnablePin, ModbusRTUTxEnableMode txEnableMode);
    void task();
};

class TelnetModbusBridge : public ModbusBridge {
public:
    TelnetModbusBridge();
    void begin(Stream& rtuStream, int16_t txEnablePin, ModbusRTUTxEnableMode txEnableMode);
    void task();
};
