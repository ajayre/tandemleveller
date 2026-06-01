// GNSS interface and processing

#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include "GNSS.h"

// NMEA 0183 special characters
#define LF 0x0A
#define CR 0x0D

// GGA/VTG: talker + comma-separated data fields (split buffer)
#define MAX_GGA_FIELDS 16

// Nautical miles per hour (knots) to km/h
#define NMEA_KNOTS_TO_KPH 1.852

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Minimum speed to feed COG into the smoother (~0.3 MPH).
static constexpr double MIN_COG_SAMPLE_KPH = 0.5;
// Below this speed, reject COG samples that snap to 0° (typical when stopped).
static constexpr double ZERO_COG_REJECT_KPH = 1.0;
// Treat COG within this many degrees of 0/360 as a north / snap-to-zero reading.
static constexpr double ZERO_COG_TOLERANCE_DEG = 0.5;
// Reject low-speed COG outliers this far from the current smoothed heading.
static constexpr double LOW_SPEED_OUTLIER_KPH = 5.0;
static constexpr double LOW_SPEED_OUTLIER_DEG = 25.0;
// Circular EMA on GNSS COG (lower = smoother; tuned for 1–3 MPH grading).
static constexpr double kGnssLpAlphaHeading = 0.18;


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

// Wraps course_deg into [0, 360).
double GNSS::NormalizeCourseDeg
  (
  double course_deg                 // raw course in degrees
  ) const
{
  double h = course_deg;
  while (h < 0.0)
  {
    h += 360.0;
  }
  while (h >= 360.0)
  {
    h -= 360.0;
  }
  return h;
}

// Returns true if a VTG/RMC COG sample should update the smoother.
bool GNSS::CourseSampleAccepted
  (
  double speed_kph,                 // reported speed in km/h
  bool speed_valid,                 // true when speed field was present
  bool course_valid,                // true when course field was present
  double course_deg,                // raw course in degrees
  bool track_valid,                 // true once a smoothed COG exists
  double smoothed_deg               // current smoothed COG in degrees
  ) const
{
  if (!course_valid || !speed_valid || speed_kph < MIN_COG_SAMPLE_KPH)
  {
    return false;
  }

  const double course = NormalizeCourseDeg(course_deg);
  const bool near_zero =
    course < ZERO_COG_TOLERANCE_DEG
    || course > (360.0 - ZERO_COG_TOLERANCE_DEG);

  if (near_zero)
  {
    if (speed_kph < ZERO_COG_REJECT_KPH)
    {
      return false;
    }
    if (track_valid)
    {
      const double smoothed = NormalizeCourseDeg(smoothed_deg);
      const bool smoothed_near_zero =
        smoothed < ZERO_COG_TOLERANCE_DEG
        || smoothed > (360.0 - ZERO_COG_TOLERANCE_DEG);
      if (!smoothed_near_zero)
      {
        return false;
      }
    }
  }

  return true;
}

// Applies one accepted COG sample to the circular EMA; updates pLoc->TrackMagneticDeg.
void GNSS::ApplySmoothedCourse
  (
  gnss_track_smooth_t *pSmooth,     // per-receiver smoother state
  gnss_location_t *pLoc,            // location record to update
  double course_deg                 // accepted raw course in degrees
  )
{
  const double course = NormalizeCourseDeg(course_deg);
  const double hr = course * (M_PI / 180.0);
  const double cn = cos(hr);
  const double sn = sin(hr);

  if (!pSmooth->init)
  {
    pSmooth->h_cos = cn;
    pSmooth->h_sin = sn;
    pSmooth->init = true;
    pLoc->TrackMagneticDeg = course;
    pLoc->TrackValid = 1;
    return;
  }

  pSmooth->h_cos = kGnssLpAlphaHeading * cn
    + (1.0 - kGnssLpAlphaHeading) * pSmooth->h_cos;
  pSmooth->h_sin = kGnssLpAlphaHeading * sn
    + (1.0 - kGnssLpAlphaHeading) * pSmooth->h_sin;

  double h_deg = atan2(pSmooth->h_sin, pSmooth->h_cos) * (180.0 / M_PI);
  pLoc->TrackMagneticDeg = NormalizeCourseDeg(h_deg);
  pLoc->TrackValid = 1;
}

// Returns the COG smoother state for the receiver identified by PGN.
gnss_track_smooth_t *GNSS::TrackSmoothForPgn
  (
  pgn_t PGN                         // NMEA source PGN (tractor / front / rear)
  )
{
  switch (PGN)
  {
    case PGN_FRONT_NMEA: return &FrontTrackSmooth;
    case PGN_REAR_NMEA:  return &RearTrackSmooth;
    default:             return &TractorTrackSmooth;
  }
}

// Updates speed always; feeds accepted COG samples into the circular EMA smoother.
void GNSS::UpdateSpeedAndSmoothedCourse
  (
  gnss_track_smooth_t *pSmooth,     // per-receiver smoother state
  gnss_location_t *pLoc,            // location record to update
  double speed_kph,                 // reported speed in km/h
  bool speed_valid,                 // true when speed field was present
  double course_deg,                // raw course in degrees
  bool course_valid                 // true when course field was present
  )
{
  if (speed_valid)
  {
    pLoc->SpeedKph = speed_kph;
  }

  if (!CourseSampleAccepted(
        speed_kph,
        speed_valid,
        course_valid,
        course_deg,
        pLoc->TrackValid != 0,
        pLoc->TrackMagneticDeg))
  {
    return;
  }

  const double course = NormalizeCourseDeg(course_deg);

  if (pSmooth->init && speed_kph < LOW_SPEED_OUTLIER_KPH)
  {
    double diff = fabs(course - pLoc->TrackMagneticDeg);
    if (diff > 180.0)
    {
      diff = 360.0 - diff;
    }
    if (diff > LOW_SPEED_OUTLIER_DEG)
    {
      return;
    }
  }

  ApplySmoothedCourse(pSmooth, pLoc, course);
}

// Rebuilds a VTG sentence from smoothed speed/course in pLoc; returns length or -1 on error.
int GNSS::FormatVtgSentence
  (
  char *out,                        // output buffer for rebuilt sentence
  size_t outLen,                    // size of out in bytes
  const char *talker,               // VTG talker id field (e.g. "GNVTG")
  const gnss_location_t *pLoc,      // location with smoothed speed/course
  const char *fields[],             // parsed fields from the original VTG
  int nfields                       // number of parsed fields
  ) const
{
  const double sogKn = pLoc->SpeedKph / (double)NMEA_KNOTS_TO_KPH;
  int o;

  if (pLoc->TrackValid)
  {
    const double cog = pLoc->TrackMagneticDeg;
    o = snprintf(out, outLen, "$%s,%.2f,T,%.2f,M,%.2f,N,%.2f,K",
      talker, cog, cog, sogKn, pLoc->SpeedKph);
  }
  else
  {
    o = snprintf(out, outLen, "$%s,,T,,M,%.2f,N,%.2f,K",
      talker, sogKn, pLoc->SpeedKph);
  }

  if (o < 0 || (size_t)o >= outLen)
  {
    return -1;
  }

  if (nfields > 9 && fields[9][0] != '\0')
  {
    const int add = snprintf(out + o, outLen - (size_t)o, ",%s", fields[9]);
    if (add < 0 || (size_t)add >= outLen - (size_t)o)
    {
      return -1;
    }
    o += add;
  }

  const uint8_t newCs = NMEAChecksumBytes(out + 1, out + o);
  const int fin = snprintf(out + o, outLen - (size_t)o, "*%02X\r\n", (unsigned)newCs);
  if (fin < 0 || (size_t)fin >= outLen - (size_t)o)
  {
    return -1;
  }

  return o + fin;
}

// Handles paths: (1) unknown sentences — pass through unchanged;
// (2) VTG / RMC — checksum, fill speed/track; VTG forwarded with smoothed COG;
// (3) GGA — checksum, fill fix, quality, HDOP, rebuild GGA with formatted lat/lon for callback.
void GNSS::ProcessNMEASentence
  (
  pgn_t PGN,
  const char *sentence,
  uint8_t length
  )
{
  // fixme - remove
  //Serial.print(sentence);

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

  // (2) VTG only: fields[] are VTG columns, not GGA. Update speed/course; forward filtered VTG.
  if (isVtg)
  {
    // $--VTG,cogTrue,T,cogMag,M,sogKn,N,sogKph,K — indices 1..8 after talker in fields[0]
    double speed_kph = 0.0;
    bool speed_valid = false;
    if (nfields > 7 && fields[7][0] != '\0')
    {
      speed_kph = strtod(fields[7], NULL);
      speed_valid = true;
    }
    else if (nfields > 5 && fields[5][0] != '\0')
    {
      speed_kph = strtod(fields[5], NULL) * (double)NMEA_KNOTS_TO_KPH;
      speed_valid = true;
    }

    double course_deg = 0.0;
    bool course_valid = false;
    if (nfields > 3 && fields[3][0] != '\0')
    {
      course_deg = strtod(fields[3], NULL);
      course_valid = true;
    }
    else if (nfields > 1 && fields[1][0] != '\0')
    {
      course_deg = strtod(fields[1], NULL);
      course_valid = true;
    }

    UpdateSpeedAndSmoothedCourse(
      TrackSmoothForPgn(PGN),
      pLoc,
      speed_kph,
      speed_valid,
      course_deg,
      course_valid);

    if (GNSSReceivedNMEACallback != NULL)
    {
      char out[MAX_NMEA_LENGTH];
      const int o = FormatVtgSentence(out, sizeof(out), fields[0], pLoc, fields, nfields);
      if (o > 0)
      {
        GNSSReceivedNMEACallback(PGN, out, (uint8_t)o);
      }
      else
      {
        GNSSReceivedNMEACallback(PGN, (char *)sentence, length);
      }
    }

    return;
  }

  // RMC: speed (knots) and course (true) when VTG not present or for redundancy
  if (isRmc)
  {
    // $--RMC,time,status,lat,N,lon,W,sogKn,cogT,date,magvar* — need status A and sog/cog
    if (nfields > 8 && fields[2][0] == 'A')
    {
      double speed_kph = 0.0;
      bool speed_valid = false;
      if (fields[7][0] != '\0')
      {
        speed_kph = strtod(fields[7], NULL) * (double)NMEA_KNOTS_TO_KPH;
        speed_valid = true;
      }

      double course_deg = 0.0;
      bool course_valid = false;
      if (fields[8][0] != '\0')
      {
        course_deg = strtod(fields[8], NULL);
        course_valid = true;
      }

      UpdateSpeedAndSmoothedCourse(
      TrackSmoothForPgn(PGN),
      pLoc,
      speed_kph,
      speed_valid,
      course_deg,
      course_valid);
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

  uint32_t utc_ms = ParseGgaUtcTimeMs(fields[1]);
  if (utc_ms > 0)
  {
    int32_t new_offset = (int32_t)(pLoc->LastFixTimeMs - utc_ms);
    if (!UtcOffsetValid)
    {
      UtcToMillisOffset = new_offset;
      UtcOffsetValid = true;
    }
    else
    {
      UtcToMillisOffset += (int32_t)((double)(new_offset - UtcToMillisOffset) * 0.1);
    }
    pLoc->FixEpochMs = (uint32_t)((int32_t)utc_ms + UtcToMillisOffset);
  }
  else
  {
    pLoc->FixEpochMs = pLoc->LastFixTimeMs;
  }

  // if tractor then store values before fusing
  if (PGN == PGN_TRACTOR_NMEA)
  {
    RawTractorLocation.Latitude         = pLoc->Latitude;
    RawTractorLocation.Longitude        = pLoc->Longitude;
    RawTractorLocation.Altitude         = pLoc->Altitude;
    RawTractorLocation.SpeedKph         = pLoc->SpeedKph;
    RawTractorLocation.TrackMagneticDeg = pLoc->TrackMagneticDeg;
    RawTractorLocation.FixQuality       = pLoc->FixQuality;
    RawTractorLocation.Hdop             = pLoc->Hdop;
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

// parses GGA UTC time field (hhmmss.ss) to milliseconds since midnight
uint32_t GNSS::ParseGgaUtcTimeMs
  (
  const char *field           // GGA field 1 contents
  ) const
{
  if (field == NULL || field[0] == '\0')
  {
    return 0;
  }
  double raw = strtod(field, NULL);
  if (raw < 0.001)
  {
    return 0;
  }
  int hms = (int)raw;
  int h = hms / 10000;
  int m = (hms / 100) % 100;
  int s = hms % 100;
  double frac = raw - (double)hms;
  return (uint32_t)((uint32_t)h * 3600000UL
    + (uint32_t)m * 60000UL
    + (uint32_t)s * 1000UL
    + (uint32_t)(frac * 1000.0));
}

// constructor
GNSS::GNSS
  (
  void
  )
  : UtcToMillisOffset(0)
  , UtcOffsetValid(false)
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
