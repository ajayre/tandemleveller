// GNSS interface and processing

#include <Arduino.h>
#include <stdlib.h>
#include "GNSS.h"

// NMEA 0183 special characters
#define LF 0x0A
#define CR 0x0D

// GGA/VTG: talker + comma-separated data fields (split buffer)
#define MAX_GGA_FIELDS 16

// Nautical miles per hour (knots) to km/h
#define NMEA_KNOTS_TO_KPH 1.852


///////////////////////////////////////////////////////////////////////////////////
// PRIVATE FUNCTIONS

// processes a byte read from a GNSS stream
void GNSS::ProcessGNSSByte
  (
  int RxByte,                 // byte to process
  gnss_reader_t *pReader,     // reader to use
  pgn_t PGN                   // PGN to use when sending
  )
{
  // byte received
  if (RxByte != -1)
  {
    // waiting to sync, got start of sentence
    if (!pReader->Synced && (RxByte == '$'))
    {
      pReader->Synced = true;
      pReader->NextWritePos = 0;
      pReader->Buffer[pReader->NextWritePos++] = RxByte;
    }
    // synced, store character
    else if (pReader->Synced)
    {
      pReader->Buffer[pReader->NextWritePos++] = (char)RxByte;

      // if end of sentence
      if (RxByte == LF)
      {
        // add null terminator
        pReader->Buffer[pReader->NextWritePos++] = NULL;
        
        // process the sentence
        ProcessNMEASentence(PGN, pReader->Buffer, strlen(pReader->Buffer));

        // start again
        pReader->Synced = false;
      }
      // sentence too long - must be mangled, discard
      else if (pReader->NextWritePos == MAX_NMEA_LENGTH)
      {
        // fixme - remove
        Serial.println("Mangled NMEA");
        pReader->Buffer[pReader->NextWritePos++] = NULL;
        Serial.print("  ");
        Serial.print(pReader->Buffer);

        pReader->Synced = false;
      }
    }
  }
}

// XOR checksum for NMEA 0183: bytes from first after '$' up to (exclusive) '*'
uint8_t GNSS::NMEAChecksumBytes
  (
  const char *startAfterDollar,
  const char *endBeforeStar
  )
{
  uint8_t cs = 0;
  for (const char *p = startAfterDollar; p < endBeforeStar; p++)
  {
    cs ^= (uint8_t)(*p);
  }
  return cs;
}

int GNSS::NMEAHexNibble
  (
  char c
  )
{
  if (c >= '0' && c <= '9')
  {
    return (int)(c - '0');
  }
  if (c >= 'A' && c <= 'F')
  {
    return (int)(c - 'A' + 10);
  }
  if (c >= 'a' && c <= 'f')
  {
    return (int)(c - 'a' + 10);
  }
  return -1;
}

// Split payload (between '$' and '*') on commas; writes '\0' into payload; returns field count
int GNSS::NMEASplitFields
  (
  char *payload,
  char *fields[],
  int maxFields
  )
{
  int n = 0;
  fields[n++] = payload;
  for (char *p = payload; *p != '\0' && n < maxFields; p++)
  {
    if (*p == ',')
    {
      *p = '\0';
      if (n < maxFields)
      {
        fields[n++] = p + 1;
      }
    }
  }
  return n;
}

// Parse NMEA ddmm.mmmmm (lat) or dddmm.mmmmm (lon) field to signed decimal degrees using hemisphere
double GNSS::NMEAParseLatLonToDecimal
  (
  const char *dmField,
  const char *hemi          // "N"/"S" or "E"/"W"
  )
{
  if (dmField == NULL || *dmField == '\0')
  {
    return 0.0;
  }
  double dm = strtod(dmField, NULL);
  int ideg = (int)(dm / 100.0);
  double minutes = dm - (double)ideg * 100.0;
  double dec = (double)ideg + minutes / 60.0;
  if (hemi != NULL)
  {
    char h = hemi[0];
    if (h == 'S' || h == 's' || h == 'W' || h == 'w')
    {
      dec = -fabs(dec);
    }
    else
    {
      dec = fabs(dec);
    }
  }
  return dec;
}

// Append minutes as mm.mmmmmmm (2 integer digits + 7 fractional digits, no spaces) after degree digits in dmOut
int GNSS::NMEAAppendMinutes
  (
  char *dmOut,
  size_t dmLen,
  size_t offset,
  double minutes
  )
{
  if (minutes < 0.0)
  {
    minutes = 0.0;
  }
  if (minutes >= 60.0)
  {
    minutes = 59.9999999;
  }
  long scaled = lround(minutes * 10000000.0);
  int minInt = (int)(scaled / 10000000L);
  int minFrac = (int)(scaled % 10000000L);
  if (minInt > 59)
  {
    minInt = 59;
    minFrac = 9999999;
  }
  int n2 = snprintf(dmOut + offset, dmLen - offset, "%02d.%07d", minInt, minFrac);
  if (n2 < 0 || offset + (size_t)n2 >= dmLen)
  {
    return -1;
  }
  return (int)(offset + (size_t)n2);
}

// Format signed decimal latitude to ddmm.mmmmmmm (7 dp on minutes) and N/S
int GNSS::NMEAFormatLatitude
  (
  double latDec,
  char *dmOut,
  size_t dmLen,
  char latNS[2]
  )
{
  double a = fabs(latDec);
  int deg = (int)floor(a + 1e-9);
  if (deg > 90)
  {
    deg = 90;
  }
  double min = (a - (double)deg) * 60.0;
  latNS[0] = (latDec < 0.0) ? 'S' : 'N';
  latNS[1] = '\0';
  int n = snprintf(dmOut, dmLen, "%02d", deg);
  if (n < 0 || (size_t)n >= dmLen)
  {
    return -1;
  }
  return NMEAAppendMinutes(dmOut, dmLen, (size_t)n, min);
}

// Format signed decimal longitude to dddmm.mmmmmmm (7 dp on minutes) and E/W
int GNSS::NMEAFormatLongitude
  (
  double lonDec,
  char *dmOut,
  size_t dmLen,
  char lonEW[2]
  )
{
  double a = fabs(lonDec);
  int deg = (int)floor(a + 1e-9);
  if (deg > 180)
  {
    deg = 180;
  }
  double min = (a - (double)deg) * 60.0;
  lonEW[0] = (lonDec < 0.0) ? 'W' : 'E';
  lonEW[1] = '\0';
  int n = snprintf(dmOut, dmLen, "%03d", deg);
  if (n < 0 || (size_t)n >= dmLen)
  {
    return -1;
  }
  return NMEAAppendMinutes(dmOut, dmLen, (size_t)n, min);
}

// Handles paths: (1) unknown sentences — pass through unchanged;
// (2) VTG / RMC — checksum, fill speed/track; pass through;
// (3) GGA — checksum, fill fix, quality, HDOP, rebuild GGA with formatted lat/lon for callback.
void GNSS::ProcessNMEASentence
  (
  pgn_t PGN,
  const char *sentence,
  uint8_t length
  )
{
  // fixme - remove
  Serial.print(sentence);

  // these messages are not forwarded because we use GGA instead
  if ((strncmp(sentence, "$GPGLL,", 7) == 0) || (strncmp(sentence, "$GNGLL,", 7) == 0))
  {
    return;
  }

  bool isGga = (strncmp(sentence, "$GPGGA,", 7) == 0) ||
               (strncmp(sentence, "$GNGGA,", 7) == 0);
  bool isVtg = (strncmp(sentence, "$GPVTG,", 7) == 0) ||
               (strncmp(sentence, "$GNVTG,", 7) == 0);
  bool isRmc = (strncmp(sentence, "$GPRMC,", 7) == 0) ||
               (strncmp(sentence, "$GNRMC,", 7) == 0);

  // (1) Not GGA, VTG, or RMC: no parsing, forward raw sentence
  if (!isGga && !isVtg && !isRmc)
  {
    if (GNSSReceivedNMEACallback != NULL)
    {
      GNSSReceivedNMEACallback(PGN, (char *)sentence, length);
    }

    return;
  }

  // GGA, VTG, and RMC share: verify checksum and split comma fields (different field layouts below).
  const char *star = strchr(sentence, '*');
  if (star == NULL || star[1] == '\0' || star[2] == '\0')
  {
    return;
  }

  int hi = NMEAHexNibble(star[1]);
  int lo = NMEAHexNibble(star[2]);
  if (hi < 0 || lo < 0)
  {
    return;
  }
  uint8_t wantCs = (uint8_t)((hi << 4) | lo);
  uint8_t gotCs = NMEAChecksumBytes(sentence + 1, star);
  if (gotCs != wantCs)
  {
    return;
  }

  char work[MAX_NMEA_LENGTH];
  if (length >= sizeof(work))
  {
    return;
  }
  memcpy(work, sentence, length);
  work[length] = '\0';

  char *wstar = strchr(work, '*');
  if (wstar == NULL)
  {
    return;
  }
  *wstar = '\0';

  char *fields[MAX_GGA_FIELDS];
  int nfields = NMEASplitFields(work + 1, fields, MAX_GGA_FIELDS);

  gnss_location_t *pLoc;
  switch (PGN)
  {
    default:
    case PGN_TRACTOR_NMEA: pLoc = &TractorLocation;      break;
    case PGN_FRONT_NMEA:   pLoc = &FrontScraperLocation; break;
    case PGN_REAR_NMEA:    pLoc = &RearScraperLocation;  break;
  }

  // (2) VTG only: fields[] are VTG columns, not GGA. Update speed/course; send original sentence.
  if (isVtg)
  {
    // $--VTG,cogTrue,T,cogMag,M,sogKn,N,sogKph,K — indices 1..8 after talker in fields[0]
    if (nfields > 7 && fields[7][0] != '\0')
    {
      pLoc->SpeedKph = strtod(fields[7], NULL);
    }
    else if (nfields > 5 && fields[5][0] != '\0')
    {
      pLoc->SpeedKph = strtod(fields[5], NULL) * (double)NMEA_KNOTS_TO_KPH;
    }

    if (nfields > 3 && fields[3][0] != '\0')
    {
      pLoc->TrackMagneticDeg = strtod(fields[3], NULL);
    }
    else if (nfields > 1 && fields[1][0] != '\0')
    {
      pLoc->TrackMagneticDeg = strtod(fields[1], NULL);
    }

    if (GNSSReceivedNMEACallback != NULL)
    {
      GNSSReceivedNMEACallback(PGN, (char *)sentence, length);
    }

    return;
  }

  // RMC: speed (knots) and course (true) when VTG not present or for redundancy
  if (isRmc)
  {
    // $--RMC,time,status,lat,N,lon,W,sogKn,cogT,date,magvar* — need status A and sog/cog
    if (nfields > 8 && fields[2][0] == 'A')
    {
      if (fields[7][0] != '\0')
      {
        pLoc->SpeedKph = strtod(fields[7], NULL) * (double)NMEA_KNOTS_TO_KPH;
      }
      if (fields[8][0] != '\0')
      {
        pLoc->TrackMagneticDeg = strtod(fields[8], NULL);
      }
    }

    if (GNSSReceivedNMEACallback != NULL)
    {
      GNSSReceivedNMEACallback(PGN, (char *)sentence, length);
    }

    return;
  }

  // (3) GGA only: fields[] are GGA columns (lat/lon/quality/altitude, etc.)
  if (nfields < 6)
  {
    return;
  }

  double lat = NMEAParseLatLonToDecimal(fields[2], fields[3]);
  double lon = NMEAParseLatLonToDecimal(fields[4], fields[5]);

  pLoc->Latitude = lat;
  pLoc->Longitude = lon;

  int qual = 0;
  if (nfields > 6 && fields[6][0] != '\0')
  {
    qual = atoi(fields[6]);
  }
  pLoc->FixQuality = qual;
  pLoc->Hdop = 0.0;
  if (nfields > 8 && fields[8][0] != '\0')
  {
    pLoc->Hdop = strtod(fields[8], NULL);
  }
  switch (qual)
  {
    case 4:
      pLoc->RtkStatus = GNSS_RTK_FIX;
      break;
    case 5:
      pLoc->RtkStatus = GNSS_RTK_FLOAT;
      break;
    default:
      pLoc->RtkStatus = GNSS_RTK_NONE;
      break;
  }

  pLoc->Altitude = 0.0;
  if (nfields > 9 && fields[9][0] != '\0')
  {
    pLoc->Altitude = strtod(fields[9], NULL);
  }

  pLoc->LastFixTimeValid = 1;
  pLoc->LastFixTimeMs = millis();

  // if tractor then store values before fusing
  if (PGN == PGN_TRACTOR_NMEA)
  {
    RawTractorLocation.Latitude        = pLoc->Latitude;
    RawTractorLocation.Longitude       = pLoc->Longitude;
    RawTractorLocation.Altitude        = pLoc->Altitude;
    RawTractorLocation.SpeedKph        = pLoc->SpeedKph;
    RawTractorLocation.TrackMagneticDeg = pLoc->TrackMagneticDeg;
    RawTractorLocation.FixQuality      = pLoc->FixQuality;
    RawTractorLocation.Hdop            = pLoc->Hdop;
  }

  if (GNSSRequestFuseCallback != NULL)
  {
    GNSSRequestFuseCallback(PGN, pLoc);
  }

  char latDm[16];
  char latNS[2];
  char lonDm[16];
  char lonEW[2];

  if (NMEAFormatLatitude(pLoc->Latitude, latDm, sizeof(latDm), latNS) < 0 ||
      NMEAFormatLongitude(pLoc->Longitude, lonDm, sizeof(lonDm), lonEW) < 0)
  {
    return;
  }

  char out[MAX_NMEA_LENGTH];
  int o = snprintf(out, sizeof(out), "$%s,%s,%s,%s,%s,%s",
                     fields[0], fields[1], latDm, latNS, lonDm, lonEW);
  if (o < 0 || o >= (int)sizeof(out))
  {
    return;
  }

  for (int f = 6; f < nfields; f++)
  {
    int add;
    if (f == 9)
    {
      char altStr[24];
      snprintf(altStr, sizeof(altStr), "%.3f", pLoc->Altitude);
      add = snprintf(out + o, sizeof(out) - (size_t)o, ",%s", altStr);
    }
    else
    {
      add = snprintf(out + o, sizeof(out) - (size_t)o, ",%s", fields[f]);
    }
    if (add < 0 || (size_t)add >= sizeof(out) - (size_t)o)
    {
      return;
    }
    o += add;
  }

  uint8_t newCs = NMEAChecksumBytes(out + 1, out + o);
  int fin = snprintf(out + o, sizeof(out) - (size_t)o, "*%02X\r\n", (unsigned)newCs);
  if (fin < 0 || (size_t)fin >= sizeof(out) - (size_t)o)
  {
    return;
  }
  o += fin;

  // send the sentence
  if (GNSSReceivedNMEACallback != NULL)
  {
    GNSSReceivedNMEACallback(PGN, out, (uint8_t)o);
  }
}


///////////////////////////////////////////////////////////////////////////////////
// PUBLIC FUNCTIONS

// constructor
GNSS::GNSS
  (
  void
  )
{
}

// connect to the GNSS receivers
void GNSS::Connect
  (
  void
  )
{
  // tractor GNSS
  Serial6.begin(115200);
  // front scraper GNSS
  Serial7.begin(115200);
  // rear scraper GNSS
  Serial8.begin(115200);
}

// sets callback functions
void GNSS::SetCallbacks
  (
  gnss_received_nmea_callback_t _GNSSReceivedNMEACallback,
  gnss_request_fuse_callback_t _GNSSRequestFuseCallback
  )
{
  GNSSReceivedNMEACallback = _GNSSReceivedNMEACallback;
  GNSSRequestFuseCallback = _GNSSRequestFuseCallback;
}

// processes the GNSS serial streams
void GNSS::Process
  (
  void
  )
{
  int RxByte;

  // process tractor GNSS
  RxByte = Serial6.read();
  ProcessGNSSByte(RxByte, &TractorGNSS, PGN_TRACTOR_NMEA);

  // process front scraper GNSS
  RxByte = Serial7.read();
  ProcessGNSSByte(RxByte, &FrontScraperGNSS, PGN_FRONT_NMEA);
  
  // process rear scraper GNSS
  RxByte = Serial8.read();
  ProcessGNSSByte(RxByte, &RearScraperGNSS, PGN_REAR_NMEA);
}
