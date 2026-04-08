// interface to AgGrade

#include "AgGrade.h"
#include "UDPTransfer.h"

// constructor
AgGrade::AgGrade
  (
    UDPTransfer *_pUdpTransfer
  )
{
  this->pUdpTransfer = _pUdpTransfer;
}

// sends status value over UDP using packet framing
void AgGrade::SendStatus
  (
  pgnpacket_t *pStatus
  )
{
  // Pack the PGN
  pUdpTransfer->packet.txBuff[0] = (uint8_t)(pStatus->PGN & 0xFF);
  pUdpTransfer->packet.txBuff[1] = (uint8_t)((pStatus->PGN >> 8) & 0xFF);

  // Pack the data
  for (int b = 0; b < MAX_PGN_LEN; b++)
  {
    pUdpTransfer->packet.txBuff[2 + b] = pStatus->Data[b];
  }

  // Send the packet
  pUdpTransfer->sendData(MAX_PGN_LEN + 2);
}
