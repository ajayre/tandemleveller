/*
 * UDPTransfer.h
 * 
 * UDP wrapper for reliable packet-based communication.
 * Provides the same interface as SerialTransfer but for UDP.
 * 
 * Usage:
 *   1. Create a UDPTransfer instance
 *   2. Call begin() with your EthernetUDP instance
 *   3. Use txObj() to pack data into the transmit buffer
 *   4. Call sendData() to transmit
 *   5. Call available() to check for received packets
 *   6. Use rxObj() to unpack received data
 */

#ifndef UDP_TRANSFER_H
#define UDP_TRANSFER_H

#include <Arduino.h>
#include <NativeEthernetUdp.h>
#include "Packet.h"

class UDPTransfer
{
public:
    // Public packet object for direct buffer access
    Packet packet;
    
    UDPTransfer();
    
    // Initialize with UDP instance
    void begin(EthernetUDP& udp, uint32_t timeout = PACKET_RX_TIMEOUT_MS);
    
    // Set the remote address and port for sending
    void setRemote(IPAddress ip, uint16_t port);
    
    // --- Transmit Functions ---
    
    // Send data from the packet buffer
    // Returns number of bytes sent
    uint8_t sendData(uint16_t messageLen, uint8_t packetId = 0);
    
    // Pack an object and send it in one call
    template <typename T>
    uint8_t sendDatum(const T& val, uint16_t len = 0)
    {
        if (len == 0)
            len = sizeof(val);
        
        return sendData(packet.txObj(val, 0, len));
    }
    
    // Copy data into the transmit buffer
    template <typename T>
    uint16_t txObj(const T& val, uint16_t index = 0, uint16_t len = 0)
    {
        if (len == 0)
            len = sizeof(val);
        
        const uint8_t* ptr = (const uint8_t*)&val;
        uint16_t maxIndex = (len + index > PACKET_MAX_PAYLOAD) ? PACKET_MAX_PAYLOAD : (len + index);
        
        for (uint16_t i = index; i < maxIndex; i++)
        {
            packet.txBuff[i] = ptr[i - index];
        }
        
        return maxIndex;
    }
    
    // --- Receive Functions ---
    
    // Check for and parse incoming packets
    // Returns bytes read when a complete packet is received
    uint8_t available();
    
    // Get the ID of the current packet
    uint8_t currentPacketId() const { return packet.currentPacketId(); }
    
    // Get the current status
    int8_t status() const { return _status; }
    
    // Get number of bytes in the last received packet
    uint8_t bytesRead() const { return _bytesRead; }
    
    // Copy data from the receive buffer into an object
    template <typename T>
    uint16_t rxObj(T& val, uint16_t index = 0, uint16_t len = 0)
    {
        if (len == 0)
            len = sizeof(val);
        
        uint8_t* ptr = (uint8_t*)&val;
        uint16_t maxIndex = (len + index > PACKET_MAX_PAYLOAD) ? PACKET_MAX_PAYLOAD : (len + index);
        
        for (uint16_t i = index; i < maxIndex; i++)
        {
            ptr[i - index] = packet.rxBuff[i];
        }
        
        return maxIndex;
    }
    
    // Reset the packet handler
    void reset();
    
private:
    EthernetUDP* _udp;
    IPAddress _remoteIP;
    uint16_t _remotePort;
    uint8_t _bytesRead;
    int8_t _status;
    
    // Buffer for receiving raw UDP data
    uint8_t _udpBuffer[PACKET_MAX_PAYLOAD + PACKET_PREAMBLE_SIZE + PACKET_POSTAMBLE_SIZE];
};

#endif // UDP_TRANSFER_H
