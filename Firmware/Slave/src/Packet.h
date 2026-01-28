/*
 * Packet.h
 * 
 * Packet framing for reliable data transmission over UDP.
 * Based on the SerialTransfer library protocol.
 * 
 * Packet Format:
 *   [START_BYTE][ID][OVERHEAD][LENGTH][PAYLOAD...][CRC][STOP_BYTE]
 *   
 *   START_BYTE: 0x7E - marks beginning of packet
 *   ID:         Packet identifier/type (0-255)
 *   OVERHEAD:   Position of first stuffed byte (0xFF if none)
 *   LENGTH:     Payload length (1-254 bytes)
 *   PAYLOAD:    Actual data (with byte stuffing applied)
 *   CRC:        CRC-8 checksum of payload
 *   STOP_BYTE:  0x81 - marks end of packet
 */

#ifndef PACKET_H
#define PACKET_H

#include <Arduino.h>
#include "PacketCRC.h"

// Packet framing constants
#define PACKET_START_BYTE     0x7E
#define PACKET_STOP_BYTE      0x81
#define PACKET_PREAMBLE_SIZE  4
#define PACKET_POSTAMBLE_SIZE 2
#define PACKET_MAX_PAYLOAD    0xFE

// Status codes
enum PacketStatus
{
    PACKET_CONTINUE        = 3,
    PACKET_NEW_DATA        = 2,
    PACKET_NO_DATA         = 1,
    PACKET_CRC_ERROR       = 0,
    PACKET_PAYLOAD_ERROR   = -1,
    PACKET_STOP_BYTE_ERROR = -2,
    PACKET_STALE_ERROR     = -3
};

class Packet
{
public:
    Packet();
    
    // Initialize the packet handler
    void begin(uint32_t timeout = 50);
    
    // --- Transmit Functions ---
    
    // Construct a packet from the TX buffer
    // Returns the number of payload bytes included
    uint8_t constructPacket(uint16_t messageLen, uint8_t packetId = 0);
    
    // Get the preamble buffer (START_BYTE, ID, OVERHEAD, LENGTH)
    uint8_t* getPreamble() { return _preamble; }
    
    // Get the postamble buffer (CRC, STOP_BYTE)
    uint8_t* getPostamble() { return _postamble; }
    
    // Get the TX buffer
    uint8_t* getTxBuff() { return _txBuff; }
    
    // Copy data into the TX buffer at the specified index
    // Returns the next available index
    template <typename T>
    uint16_t txObj(const T& val, uint16_t index = 0, uint16_t len = 0)
    {
        if (len == 0)
            len = sizeof(val);
        
        const uint8_t* ptr = (const uint8_t*)&val;
        uint16_t maxIndex = (len + index > PACKET_MAX_PAYLOAD) ? PACKET_MAX_PAYLOAD : (len + index);
        
        for (uint16_t i = index; i < maxIndex; i++)
        {
            _txBuff[i] = ptr[i - index];
        }
        
        return maxIndex;
    }
    
    // --- Receive Functions ---
    
    // Parse a received byte
    // Returns bytes read when a complete packet is received
    uint8_t parse(uint8_t recChar, bool valid = true);
    
    // Get the RX buffer
    uint8_t* getRxBuff() { return _rxBuff; }
    
    // Get the current packet ID
    uint8_t currentPacketId() const { return _idByte; }
    
    // Get the current status
    int8_t getStatus() const { return _status; }
    
    // Copy data from the RX buffer into an object
    // Returns the next available index
    template <typename T>
    uint16_t rxObj(T& val, uint16_t index = 0, uint16_t len = 0)
    {
        if (len == 0)
            len = sizeof(val);
        
        uint8_t* ptr = (uint8_t*)&val;
        uint16_t maxIndex = (len + index > PACKET_MAX_PAYLOAD) ? PACKET_MAX_PAYLOAD : (len + index);
        
        for (uint16_t i = index; i < maxIndex; i++)
        {
            ptr[i - index] = _rxBuff[i];
        }
        
        return maxIndex;
    }
    
    // Reset the packet handler
    void reset();
    
    // --- Buffer Access ---
    
    // Direct buffer access for manual manipulation
    uint8_t txBuff[PACKET_MAX_PAYLOAD];
    uint8_t rxBuff[PACKET_MAX_PAYLOAD];
    
private:
    // Buffers
    uint8_t _txBuff[PACKET_MAX_PAYLOAD];
    uint8_t _rxBuff[PACKET_MAX_PAYLOAD];
    uint8_t _preamble[PACKET_PREAMBLE_SIZE];
    uint8_t _postamble[PACKET_POSTAMBLE_SIZE];
    
    // CRC calculator
    PacketCRC _crc;
    
    // State machine
    enum ParseState
    {
        FIND_START_BYTE,
        FIND_ID_BYTE,
        FIND_OVERHEAD_BYTE,
        FIND_PAYLOAD_LEN,
        FIND_PAYLOAD,
        FIND_CRC,
        FIND_END_BYTE
    };
    
    ParseState _state;
    
    // Tracking variables
    uint8_t _bytesToRec;
    uint8_t _payIndex;
    uint8_t _idByte;
    uint8_t _overheadByte;
    uint8_t _recOverheadByte;
    uint8_t _bytesRead;
    int8_t _status;
    
    uint32_t _packetStart;
    uint32_t _timeout;
    
    // Internal functions
    void calcOverhead(uint8_t* arr, uint8_t len);
    int16_t findLast(uint8_t* arr, uint8_t len);
    void stuffPacket(uint8_t* arr, uint8_t len);
    void unpackPacket(uint8_t* arr);
};

#endif // PACKET_H
