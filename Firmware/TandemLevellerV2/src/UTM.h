// WGS84 UTM (Transverse Mercator) — forward conversion

#ifndef _UTMH_
#define _UTMH_

#include <Arduino.h>

class UTM
{
  public:
    // WGS84 geodetic latitude/longitude (degrees, -90..90 / -180..180) to UTM
    // easting / northing (meters). Zone is 1..60 from longitude (EPSG-style).
    // Northing includes southern-hemisphere false northing (10e6 m) when lat < 0.
    // Accuracy target: well below 0.5 cm in double for typical latitudes (full TM series).
    static void LatLonToUtm
      (
      double latitude_deg,
      double longitude_deg,
      double *easting_m,
      double *northing_m,
      int *utm_zone_out
      );

  private:
    static double Wgs84Es
      (
      void
      );

    static double MeridianArcWgs84
      (
      double phi_rad
      );

    static constexpr double Wgs84A_m       = 6378137.0;
    static constexpr double Wgs84InvF      = 298.257223563;
    static constexpr double UtmScaleK0     = 0.9996;
    static constexpr double UtmFalseE_m    = 500000.0;
    static constexpr double UtmFalseN_s_m  = 10000000.0;
};

#endif // _UTMH_
