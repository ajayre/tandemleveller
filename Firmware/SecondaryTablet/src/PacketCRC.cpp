/*
 * PacketCRC.cpp
 * 
 * CRC-8 calculation for packet integrity checking.
 * Based on the SerialTransfer library protocol.
 */

#include "PacketCRC.h"

PacketCRC::PacketCRC(uint8_t polynomial)
    : _poly(polynomial)
{
    generateTable();
}

void PacketCRC::generateTable()
{
    for (uint16_t i = 0; i < 256; ++i)
    {
        int curr = i;
        for (int j = 0; j < 8; ++j)
        {
            if ((curr & 0x80) != 0)
                curr = (curr << 1) ^ _poly;
            else
                curr <<= 1;
        }
        _csTable[i] = (uint8_t)curr;
    }
}

uint8_t PacketCRC::calculate(uint8_t val)
{
    return _csTable[val];
}

uint8_t PacketCRC::calculate(const uint8_t* arr, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++)
    {
        crc = _csTable[crc ^ arr[i]];
    }
    return crc;
}
