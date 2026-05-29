/*
 * UDPTransfer.cpp
 * 
 * UDP wrapper for reliable packet-based communication.
 * Provides the same interface as SerialTransfer but for UDP.
 */

#include "UDPTransfer.h"

UDPTransfer::UDPTransfer()
    : _udp(nullptr)
    , _remoteIP(0, 0, 0, 0)
    , _remotePort(0)
    , _bytesRead(0)
    , _status(PACKET_NO_DATA)
{
    memset(_udpBuffer, 0, sizeof(_udpBuffer));
}

void UDPTransfer::begin(EthernetUDP& udp, uint32_t timeout)
{
    _udp = &udp;
    packet.begin(timeout);
}

void UDPTransfer::setRemote(IPAddress ip, uint16_t port)
{
    _remoteIP = ip;
    _remotePort = port;
}

uint8_t UDPTransfer::sendData(uint16_t messageLen, uint8_t packetId)
{
    if (_udp == nullptr)
    {
        _status = PACKET_PAYLOAD_ERROR;
        return 0;
    }
    
    // Construct the packet
    uint8_t numBytesIncl = packet.constructPacket(messageLen, packetId);
    
    // Calculate total packet size
    uint16_t totalSize = PACKET_PREAMBLE_SIZE + numBytesIncl + PACKET_POSTAMBLE_SIZE;
    
    // Build the complete packet in the UDP buffer
    uint16_t idx = 0;
    
    // Copy preamble
    uint8_t* preamble = packet.getPreamble();
    for (uint8_t i = 0; i < PACKET_PREAMBLE_SIZE; i++)
    {
        _udpBuffer[idx++] = preamble[i];
    }
    
    // Copy payload
    uint8_t* txBuff = packet.getTxBuff();
    for (uint8_t i = 0; i < numBytesIncl; i++)
    {
        _udpBuffer[idx++] = txBuff[i];
    }
    
    // Copy postamble
    uint8_t* postamble = packet.getPostamble();
    for (uint8_t i = 0; i < PACKET_POSTAMBLE_SIZE; i++)
    {
        _udpBuffer[idx++] = postamble[i];
    }
    
    // Send the packet
    _udp->beginPacket(_remoteIP, _remotePort);
    _udp->write(_udpBuffer, totalSize);
    _udp->endPacket();
    
    return numBytesIncl;
}

uint8_t UDPTransfer::available()
{
    if (_udp == nullptr)
    {
        _status = PACKET_PAYLOAD_ERROR;
        return 0;
    }
    
    // Check for incoming UDP packet
    int packetSize = _udp->parsePacket();
    
    if (packetSize > 0)
    {
        // Read the UDP packet
        int bytesReceived = _udp->read(_udpBuffer, sizeof(_udpBuffer));
        
        // Parse all bytes through the packet parser
        for (int i = 0; i < bytesReceived; i++)
        {
            _bytesRead = packet.parse(_udpBuffer[i], true);
            _status = packet.getStatus();

            if (_status == PACKET_NEW_DATA)
            {
                return _bytesRead;
            }

            // Junk bad or timed-out frames and keep scanning this datagram
            if (_status < 0)
            {
                if (_status != PACKET_STALE_ERROR)
                {
                    reset();
                }
            }
        }
    }
    else if (packet.pollReceiveTimeout())
    {
        _bytesRead = 0;
        _status = PACKET_STALE_ERROR;
    }
    
    return 0;
}

void UDPTransfer::reset()
{
    packet.reset();
    _bytesRead = 0;
    _status = PACKET_NO_DATA;
}
