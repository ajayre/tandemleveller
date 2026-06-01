// GNSS interface and processing

#ifndef _GNSSH_
#define _GNSSH_

#include "Global.h"
#include "AgGrade.h"

// data structure for reading GNSS streams
typedef struct _gnss_reader_t
{
  bool Synced;
  uint8_t NextWritePos;
  char Buffer[MAX_NMEA_LENGTH];
} gnss_reader_t;

// RTK quality from fix / GGA quality (matches fusion and common NMEA usage)
typedef enum _gnss_rtk_status_t
{
  GNSS_RTK_NONE = 0,
  GNSS_RTK_FIX,
  GNSS_RTK_FLOAT
} gnss_rtk_status_t;

typedef struct _gnss_location_t
{
  double Latitude;
  double Longitude;
  double Altitude;
  // Smoothed course over ground sent to AgGrade (EMA of VTG/RMC COG).
  double TrackMagneticDeg;
  // Non-zero once a smoothed COG has been established.
  int TrackValid;
  double SpeedKph;
  // GGA fix quality field (0 = invalid, 1 = GPS, 2 = DGPS, 4 = RTK fixed, 5 = RTK float, etc.)
  int FixQuality;
  // GGA HDOP (horizontal dilution); 0 if unknown
  double Hdop;
  gnss_rtk_status_t RtkStatus;
  int LastFixTimeValid;
  uint32_t LastFixTimeMs;
  // Estimated millis() at GNSS measurement epoch, derived from
  // GGA UTC timestamp with smoothed UTC-to-millis offset.
  uint32_t FixEpochMs;
} gnss_location_t;

typedef struct _gnss_track_smooth_t
{
  double h_cos;   // circular EMA cosine component of COG
  double h_sin;   // circular EMA sine component of COG
  bool init;      // true once the first accepted COG sample has been applied
} gnss_track_smooth_t;

// callback function type for receiving NMEA sentences
typedef void (*gnss_received_nmea_callback_t)(pgn_t PGN, char *Sentence, uint8_t Length);
// callback function for requesting sensor fusing
typedef void (*gnss_request_fuse_callback_t)(pgn_t PGN, gnss_location_t *pLocation);

class GNSS
{
  public:
    gnss_location_t TractorLocation      = { 0 };
    gnss_location_t FrontScraperLocation = { 0 };
    gnss_location_t RearScraperLocation  = { 0 };

    gnss_location_t RawTractorLocation   = { 0 };

    // constructor
    GNSS
      (
      void
      );

    // processes the GNSS serial streams
    void Process
      (
      void
      );

    // sets callback functions
    void SetCallbacks
      (
      gnss_received_nmea_callback_t GNSSReceivedNMEACallback,
      gnss_request_fuse_callback_t GNSSRequestFuseCallback
      );

    // connect to the GNSS receivers
    void Connect
      (
      void
      );

  private:
    // GNSS stream readers
    gnss_reader_t TractorGNSS      = { 0 };
    gnss_reader_t FrontScraperGNSS = { 0 };
    gnss_reader_t RearScraperGNSS  = { 0 };

    // callback functions
    gnss_received_nmea_callback_t GNSSReceivedNMEACallback = NULL;
    gnss_request_fuse_callback_t GNSSRequestFuseCallback = NULL;

    // Per-receiver circular EMA state for VTG/RMC course-over-ground smoothing.
    gnss_track_smooth_t TractorTrackSmooth = { 0 };
    gnss_track_smooth_t FrontTrackSmooth   = { 0 };
    gnss_track_smooth_t RearTrackSmooth    = { 0 };

    // XOR checksum for NMEA 0183: bytes from first after '$' up to (exclusive) '*'
    uint8_t NMEAChecksumBytes
      (
      const char *startAfterDollar,
      const char *endBeforeStar
      );

    int NMEASplitFields
      (
      char *payload,
      char *fields[],
      int maxFields
      );

    int NMEAAppendMinutes
      (
      char *dmOut,
      size_t dmLen,
      size_t offset,
      double minutes
      );

    // Format signed decimal latitude to ddmm.mmmmmmm (7 dp on minutes) and N/S
    int NMEAFormatLatitude
      (
      double latDec,
      char *dmOut,
      size_t dmLen,
      char latNS[2]
      );

    // Format signed decimal longitude to dddmm.mmmmmmm (7 dp on minutes) and E/W
    int NMEAFormatLongitude
      (
      double lonDec,
      char *dmOut,
      size_t dmLen,
      char lonEW[2]
      );

    double NMEAParseLatLonToDecimal
      (
      const char *dmField,
      const char *hemi          // "N"/"S" or "E"/"W"
      );

    int NMEAHexNibble
      (
      char c
      );

    void ProcessNMEASentence
      (
      pgn_t PGN,
      const char *sentence,
      uint8_t length
      );

    // Wraps course_deg into [0, 360).
    double NormalizeCourseDeg
      (
      double course_deg                 // raw course in degrees
      ) const;

    // Returns true if a VTG/RMC COG sample should update the smoother.
    bool CourseSampleAccepted
      (
      double speed_kph,                 // reported speed in km/h
      bool speed_valid,                 // true when speed field was present
      bool course_valid,                // true when course field was present
      double course_deg,                // raw course in degrees
      bool track_valid,                 // true once a smoothed COG exists
      double smoothed_deg               // current smoothed COG in degrees
      ) const;

    // Applies one accepted COG sample to the circular EMA; updates pLoc->TrackMagneticDeg.
    void ApplySmoothedCourse
      (
      gnss_track_smooth_t *pSmooth,     // per-receiver smoother state
      gnss_location_t *pLoc,            // location record to update
      double course_deg                 // accepted raw course in degrees
      );

    // Returns the COG smoother state for the receiver identified by PGN.
    gnss_track_smooth_t *TrackSmoothForPgn
      (
      pgn_t PGN                         // NMEA source PGN (tractor / front / rear)
      );

    // Updates speed always; feeds accepted COG samples into the circular EMA smoother.
    void UpdateSpeedAndSmoothedCourse
      (
      gnss_track_smooth_t *pSmooth,     // per-receiver smoother state
      gnss_location_t *pLoc,            // location record to update
      double speed_kph,                 // reported speed in km/h
      bool speed_valid,                 // true when speed field was present
      double course_deg,                // raw course in degrees
      bool course_valid                 // true when course field was present
      );

    // Rebuilds a VTG sentence from smoothed speed/course in pLoc.
    int FormatVtgSentence
      (
      char *out,                        // output buffer for rebuilt sentence
      size_t outLen,                    // size of out in bytes
      const char *talker,               // VTG talker id field (e.g. "GNVTG")
      const gnss_location_t *pLoc,        // location with smoothed speed/course
      const char *fields[],             // parsed fields from the original VTG
      int nfields                       // number of parsed fields
      ) const;

    // processes a byte read from a GNSS stream
    void ProcessGNSSByte
      (
      int RxByte,                 // byte to process
      gnss_reader_t *pReader,     // reader to use
      pgn_t PGN                   // PGN to use when sending
      );

    // parses GGA UTC time field (hhmmss.ss) to milliseconds since midnight
    uint32_t ParseGgaUtcTimeMs
      (
      const char *field           // GGA field 1 contents
      ) const;

    // UTC-to-millis() offset, EMA-smoothed across GGA fixes to remove
    // per-sentence parse-time jitter. The offset includes the constant GNSS
    // output latency so that FixEpochMs is a jitter-free estimate of when the
    // measurement was taken in millis() terms.
    int32_t UtcToMillisOffset;
    bool UtcOffsetValid;
};

#endif // _GNSSH_
