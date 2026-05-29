/*
 * Packet.cpp
 * 
 * Packet framing for reliable data transmission over UDP.
 * Based on the SerialTransfer library protocol.
 */

#include "Packet.h"

Packet::Packet()
    : _crc()
    , _state(FIND_START_BYTE)
    , _bytesToRec(0)
    , _payIndex(0)
    , _idByte(0)
    , _overheadByte(0)
    , _recOverheadByte(0)
    , _bytesRead(0)
    , _status(PACKET_NO_DATA)
    , _packetStart(0)
    , _timeout(PACKET_RX_TIMEOUT_MS)
{
    // Initialize preamble with start byte
    _preamble[0] = PACKET_START_BYTE;
    _preamble[1] = 0;
    _preamble[2] = 0;
    _preamble[3] = 0;
    
    // Initialize postamble with stop byte
    _postamble[0] = 0;
    _postamble[1] = PACKET_STOP_BYTE;
    
    // Clear buffers
    memset(_txBuff, 0, PACKET_MAX_PAYLOAD);
    memset(_rxBuff, 0, PACKET_MAX_PAYLOAD);
    memset(txBuff, 0, PACKET_MAX_PAYLOAD);
    memset(rxBuff, 0, PACKET_MAX_PAYLOAD);
}

void Packet::begin(uint32_t timeout)
{
    _timeout = timeout;
    reset();
}

uint8_t Packet::constructPacket(uint16_t messageLen, uint8_t packetId)
{
    uint8_t processedLen;
    
    // Copy from public buffer to internal buffer
    memcpy(_txBuff, txBuff, PACKET_MAX_PAYLOAD);
    
    if (messageLen > PACKET_MAX_PAYLOAD)
    {
        calcOverhead(_txBuff, PACKET_MAX_PAYLOAD);
        stuffPacket(_txBuff, PACKET_MAX_PAYLOAD);
        uint8_t crcVal = _crc.calculate(_txBuff, PACKET_MAX_PAYLOAD);
        
        _preamble[1] = packetId;
        _preamble[2] = _overheadByte;
        _preamble[3] = PACKET_MAX_PAYLOAD;
        
        _postamble[0] = crcVal;
        
        processedLen = PACKET_MAX_PAYLOAD;
    }
    else
    {
        calcOverhead(_txBuff, (uint8_t)messageLen);
        stuffPacket(_txBuff, (uint8_t)messageLen);
        uint8_t crcVal = _crc.calculate(_txBuff, (uint8_t)messageLen);
        
        _preamble[1] = packetId;
        _preamble[2] = _overheadByte;
        _preamble[3] = (uint8_t)messageLen;
        
        _postamble[0] = crcVal;
        
        processedLen = (uint8_t)messageLen;
    }
    
    return processedLen;
}

void Packet::discardPartialPacket()
{
    _bytesRead = 0;
    _state = FIND_START_BYTE;
    _status = PACKET_STALE_ERROR;
    _packetStart = 0;
    _payIndex = 0;
    _bytesToRec = 0;
}

bool Packet::pollReceiveTimeout()
{
    if (_packetStart == 0)
    {
        return false;
    }

    if ((millis() - _packetStart) < _timeout)
    {
        return false;
    }

    discardPartialPacket();
    return true;
}

uint8_t Packet::parse(uint8_t recChar, bool valid)
{
    const bool timedOut = pollReceiveTimeout();

    if (!valid)
    {
        _bytesRead = 0;
        if (!timedOut)
        {
            _status = PACKET_NO_DATA;
        }
        return _bytesRead;
    }

    switch (_state)
    {
        case FIND_START_BYTE:
            if (recChar == PACKET_START_BYTE)
            {
                _state = FIND_ID_BYTE;
                _packetStart = millis();
            }
            break;
            
        case FIND_ID_BYTE:
            _idByte = recChar;
            _state = FIND_OVERHEAD_BYTE;
            break;
            
        case FIND_OVERHEAD_BYTE:
            _recOverheadByte = recChar;
            _state = FIND_PAYLOAD_LEN;
            break;
            
        case FIND_PAYLOAD_LEN:
            if (recChar > 0 && recChar <= PACKET_MAX_PAYLOAD)
            {
                _bytesToRec = recChar;
                _payIndex = 0;
                _state = FIND_PAYLOAD;
            }
            else
            {
                _bytesRead = 0;
                _state = FIND_START_BYTE;
                _status = PACKET_PAYLOAD_ERROR;
                reset();
                return _bytesRead;
            }
            break;
            
        case FIND_PAYLOAD:
            if (_payIndex < _bytesToRec)
            {
                _rxBuff[_payIndex] = recChar;
                _payIndex++;
                
                if (_payIndex == _bytesToRec)
                    _state = FIND_CRC;
            }
            break;
            
        case FIND_CRC:
        {
            uint8_t calcCrc = _crc.calculate(_rxBuff, _bytesToRec);
            
            if (calcCrc == recChar)
            {
                _state = FIND_END_BYTE;
            }
            else
            {
                _bytesRead = 0;
                _state = FIND_START_BYTE;
                _status = PACKET_CRC_ERROR;
                reset();
                return _bytesRead;
            }
            break;
        }
            
        case FIND_END_BYTE:
            _state = FIND_START_BYTE;
            
            if (recChar == PACKET_STOP_BYTE)
            {
                if (!unpackPacket(_rxBuff, _bytesToRec))
                {
                    _bytesRead = 0;
                    _state = FIND_START_BYTE;
                    _status = PACKET_PAYLOAD_ERROR;
                    reset();
                    return _bytesRead;
                }

                // Copy to public buffer
                memcpy(rxBuff, _rxBuff, PACKET_MAX_PAYLOAD);
                
                _bytesRead = _bytesToRec;
                _status = PACKET_NEW_DATA;
                _packetStart = 0;
                
                return _bytesRead;
            }
            
            _bytesRead = 0;
            _status = PACKET_STOP_BYTE_ERROR;
            reset();
            return _bytesRead;
            
        default:
            reset();
            _bytesRead = 0;
            _state = FIND_START_BYTE;
            break;
    }

    _bytesRead = 0;
    _status = PACKET_CONTINUE;
    return _bytesRead;
}

void Packet::reset()
{
    memset(_txBuff, 0, PACKET_MAX_PAYLOAD);
    memset(_rxBuff, 0, PACKET_MAX_PAYLOAD);
    memset(txBuff, 0, PACKET_MAX_PAYLOAD);
    memset(rxBuff, 0, PACKET_MAX_PAYLOAD);
    
    _bytesRead = 0;
    _packetStart = 0;
    _state = FIND_START_BYTE;
}

void Packet::calcOverhead(uint8_t* arr, uint8_t len)
{
    _overheadByte = 0xFF;
    
    for (uint8_t i = 0; i < len; i++)
    {
        if (arr[i] == PACKET_START_BYTE)
        {
            _overheadByte = i;
            break;
        }
    }
}

int16_t Packet::findLast(uint8_t* arr, uint8_t len)
{
    for (int16_t i = len - 1; i >= 0; i--)
    {
        if (arr[i] == PACKET_START_BYTE)
            return i;
    }
    
    return -1;
}

void Packet::stuffPacket(uint8_t* arr, uint8_t len)
{
    int16_t refByte = findLast(arr, len);
    
    if (refByte != -1)
    {
        for (int16_t i = len - 1; i >= 0; i--)
        {
            if (arr[i] == PACKET_START_BYTE)
            {
                arr[i] = (uint8_t)(refByte - i);
                refByte = i;
            }
        }
    }
}

bool Packet::unpackPacket(uint8_t* arr, uint8_t len)
{
    if (_recOverheadByte == 0xFF)
    {
        return true;
    }

    if (len == 0 || _recOverheadByte >= len)
    {
        return false;
    }

    uint8_t testIndex = _recOverheadByte;
    uint8_t steps = 0;

    while (arr[testIndex] != 0)
    {
        if (steps >= len)
        {
            return false;
        }

        const uint8_t delta = arr[testIndex];
        if (delta == 0)
        {
            return false;
        }

        arr[testIndex] = PACKET_START_BYTE;

        const uint16_t nextIndex = (uint16_t)testIndex + (uint16_t)delta;
        if (nextIndex >= len)
        {
            return false;
        }

        testIndex = (uint8_t)nextIndex;
        steps++;
    }

    if (testIndex >= len)
    {
        return false;
    }

    arr[testIndex] = PACKET_START_BYTE;
    return true;
}
