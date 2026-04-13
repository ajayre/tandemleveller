// WGS84 UTM (Transverse Mercator) forward conversion

#include <math.h>
#include "UTM.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

///////////////////////////////////////////////////////////////////////////////////
// PRIVATE FUNCTIONS

double UTM::Wgs84Es
  (
  void
  )
{
  const double f = 1.0 / Wgs84InvF;
  return f * (2.0 - f);
}

// Meridional arc from equator to phi (radians), WGS84 (series in e^2).
double UTM::MeridianArcWgs84
  (
  double phi_rad
  )
{
  const double a = Wgs84A_m;
  const double e2 = Wgs84Es();
  const double e4 = e2 * e2;
  const double e6 = e4 * e2;
  const double e8 = e4 * e4;

  const double A0 = 1.0 - e2 / 4.0 - 3.0 * e4 / 64.0 - 5.0 * e6 / 256.0 - 175.0 * e8 / 16384.0;
  const double A2 = (3.0 * e2 / 8.0 + 3.0 * e4 / 32.0 + 45.0 * e6 / 1024.0 + 105.0 * e8 / 4096.0);
  const double A4 = (15.0 * e4 / 256.0 + 45.0 * e6 / 1024.0 + 525.0 * e8 / 16384.0);
  const double A6 = (35.0 * e6 / 3072.0 + 175.0 * e8 / 12288.0);
  const double A8 = 315.0 * e8 / 131072.0;

  const double s2 = sin(2.0 * phi_rad);
  const double s4 = sin(4.0 * phi_rad);
  const double s6 = sin(6.0 * phi_rad);
  const double s8 = sin(8.0 * phi_rad);

  return a * (A0 * phi_rad - A2 * s2 + A4 * s4 - A6 * s6 + A8 * s8);
}

///////////////////////////////////////////////////////////////////////////////////
// PUBLIC FUNCTIONS

void UTM::LatLonToUtm
  (
  double latitude_deg,
  double longitude_deg,
  double *easting_m,
  double *northing_m,
  int *utm_zone_out
  )
{
  if (easting_m == NULL || northing_m == NULL)
  {
    return;
  }

  const double es = Wgs84Es();
  const double ep2 = es / (1.0 - es);

  const double lat_deg = latitude_deg;
  double lon_deg = longitude_deg;
  while (lon_deg < -180.0)
  {
    lon_deg += 360.0;
  }
  while (lon_deg > 180.0)
  {
    lon_deg -= 360.0;
  }

  int zone = (int)floor((lon_deg + 180.0) / 6.0) + 1;
  if (zone < 1)
  {
    zone = 1;
  }
  if (zone > 60)
  {
    zone = 60;
  }

  if (utm_zone_out != NULL)
  {
    *utm_zone_out = zone;
  }

  const double lon0_deg = (double)(6 * zone - 183);
  const double phi = lat_deg * (M_PI / 180.0);
  const double lam = lon_deg * (M_PI / 180.0);
  const double lam0 = lon0_deg * (M_PI / 180.0);

  const double sp = sin(phi);
  const double cp = cos(phi);
  const double tp = tan(phi);

  const double N = Wgs84A_m / sqrt(1.0 - es * sp * sp);
  const double T = tp * tp;
  const double C = ep2 * cp * cp;
  const double A = (lam - lam0) * cp;

  const double A2 = A * A;
  const double A3 = A2 * A;
  const double A4 = A2 * A2;
  const double A5 = A4 * A;
  const double A6 = A3 * A3;

  const double M = MeridianArcWgs84(phi);

  const double x =
    UtmScaleK0 * N
      * (A + (1.0 - T + C) * A3 / 6.0
        + (5.0 - 18.0 * T + T * T + 72.0 * C - 58.0 * ep2) * A5 / 120.0)
    + UtmFalseE_m;

  const double inner =
    A2 / 2.0
    + (5.0 - T + 9.0 * C + 4.0 * C * C) * A4 / 24.0
    + (61.0 - 58.0 * T + T * T + 600.0 * C - 330.0 * ep2) * A6 / 720.0;

  double y = UtmScaleK0 * (M + N * tp * inner);

  if (lat_deg < 0.0)
  {
    y += UtmFalseN_s_m;
  }

  *easting_m = x;
  *northing_m = y;
}
