/*
 * PacketCRC.h
 * 
 * CRC-8 calculation for packet integrity checking.
 * Based on the SerialTransfer library protocol.
 */

#ifndef PACKET_CRC_H
#define PACKET_CRC_H

#include <Arduino.h>

class PacketCRC
{
public:
    PacketCRC(uint8_t polynomial = 0x9B);
    
    // Calculate CRC for a single byte
    uint8_t calculate(uint8_t val);
    
    // Calculate CRC for a byte array
    uint8_t calculate(const uint8_t* arr, uint8_t len);
    
private:
    uint8_t _poly;
    uint8_t _csTable[256];
    
    void generateTable();
};

#endif // PACKET_CRC_H
