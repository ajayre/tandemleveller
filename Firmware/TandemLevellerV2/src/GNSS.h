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

typedef struct _gnss_location_t
{
  double Latitude;
  double Longitude;
} gnss_location_t;

// callback function type for receiving NMEA sentences
typedef void (*gnss_received_nmea_callback_t)(pgn_t PGN, char *Sentence, uint8_t Length);

class GNSS
{
  public:
    gnss_location_t TractorLocation = { 0 };

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

    // sets callback function when module receives an NMEA sentence
    void SetCallback
      (
      gnss_received_nmea_callback_t GNSSReceivedNMEACallback  // function to call
      );

    // connect to the GNSS receivers
    void Connect
      (
      void
      );

  private:
    // GNSS stream readers
    gnss_reader_t TractorGNSS       = { 0 };
    gnss_reader_t FrontScraperGNSS  = { 0 };
    gnss_reader_t RearScraperGNSS   = { 0 };

    // callback function
    gnss_received_nmea_callback_t GNSSReceivedNMEACallback = NULL;

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

    // processes a byte read from a GNSS stream
    void ProcessGNSSByte
      (
      int RxByte,                 // byte to process
      gnss_reader_t *pReader,     // reader to use
      pgn_t PGN                   // PGN to use when sending
      );
};

#endif // _GNSSH_
