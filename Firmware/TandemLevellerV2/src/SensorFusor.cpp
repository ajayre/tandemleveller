// GNSS / IMU fusion

#include <math.h>
#include "SensorFusor.h"


///////////////////////////////////////////////////////////////////////////////////
// PRIVATE FUNCTIONS

// sets the magnetic declination to use
void SensorFusor::SetMagneticDeclination
  (
  uint32_t Declination               // declination in degrees x 100
  )
{
  MagneticDeclinationDeg = Declination / 100;
}

// Local tangent plane: North/East displacement (mm) and altitude delta (mm, up +).
// delta_alt_m is (fused altitude - input GNSS altitude), not ellipsoid height.
void SensorFusor::ConvertResultstoMm
  (
  double in_lat_deg,
  double in_lon_deg,
  double out_lat_deg,
  double out_lon_deg,
  double delta_alt_m
  )
{
  constexpr double earth_r_m = 6378137.0;
  constexpr double pi = 3.14159265358979323846;
  constexpr double deg_to_rad = pi / 180.0;
  const double dlat = out_lat_deg - in_lat_deg;
  const double dlon = out_lon_deg - in_lon_deg;
  const double north_mm = dlat * deg_to_rad * earth_r_m * 1000.0;
  const double east_mm =
    dlon * deg_to_rad * earth_r_m * cos(in_lat_deg * deg_to_rad) * 1000.0;
  const double up_mm = delta_alt_m * 1000.0;

  if (FuseAppliedCallback != NULL)
  {
    FuseAppliedCallback(this, (int)east_mm, (int)north_mm, (int)up_mm);
  }
}

// returns non-zero if fix has RTK fixed or float solution
int SensorFusor::FixHasRtk
  (
  const FusionGnssFix *fix               // fix to test for RTK fixed or float
  ) const
{
  return fix->rtk == GNSS_RTK_FIX || fix->rtk == GNSS_RTK_FLOAT;
}

// great-circle distance in meters between two WGS84 lat/lon points (degrees)
double SensorFusor::HaversineDistanceM
  (
  double from_lat_deg,                   // start latitude (degrees)
  double from_lon_deg,                   // start longitude (degrees)
  double to_lat_deg,                     // end latitude (degrees)
  double to_lon_deg                      // end longitude (degrees)
  ) const
{
  const double lon_r1 = from_lon_deg * DegToRad;
  const double lon_r2 = to_lon_deg * DegToRad;
  const double lat_r1 = from_lat_deg * DegToRad;
  const double lat_r2 = to_lat_deg * DegToRad;
  const double dlon = lon_r2 - lon_r1;
  const double dlat = lat_r2 - lat_r1;
  const double sdlat = sin(dlat * 0.5);
  const double sdlon = sin(dlon * 0.5);
  const double a = sdlat * sdlat
    + cos(lat_r1) * cos(lat_r2) * (sdlon * sdlon);
  const double e = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return EarthRadiusM * e;
}

// initial bearing from first point to second, radians, -pi..pi
double SensorFusor::BearingRad
  (
  double from_lat_deg,                   // start latitude (degrees)
  double from_lon_deg,                   // start longitude (degrees)
  double to_lat_deg,                     // end latitude (degrees)
  double to_lon_deg                      // end longitude (degrees)
  ) const
{
  const double lat_r1 = from_lat_deg * DegToRad;
  const double lat_r2 = to_lat_deg * DegToRad;
  const double lon_r1 = from_lon_deg * DegToRad;
  const double lon_r2 = to_lon_deg * DegToRad;
  const double y = cos(lat_r2) * sin(lon_r2 - lon_r1);
  const double x = cos(lat_r1) * sin(lat_r2)
    - sin(lat_r1) * cos(lat_r2) * cos(lon_r2 - lon_r1);
  return atan2(y, x);
}

// initial bearing in degrees (legacy lon/lat order into BearingRad)
double SensorFusor::BearingDegrees
  (
  double from_lat_deg,                   // start latitude (degrees)
  double from_lon_deg,                   // start longitude (degrees)
  double to_lat_deg,                     // end latitude (degrees)
  double to_lon_deg                      // end longitude (degrees)
  ) const
{
  return BearingRad(from_lat_deg, from_lon_deg, to_lat_deg, to_lon_deg)
    * RadToDeg;
}

double SensorFusor::NormalizeHeadingDeg
  (
  double heading_deg
  ) const
{
  double h = heading_deg;
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

// moves (*lat_deg,*lon_deg) by distance_m along heading_deg (degrees, clockwise from north)
void SensorFusor::MoveDistanceBearing
  (
  double *lat_deg,                       // in-out latitude (degrees)
  double *lon_deg,                       // in-out longitude (degrees)
  double heading_deg,                    // bearing clockwise from north (degrees)
  double distance_m                      // distance to move (meters)
  ) const
{
  const double offset = distance_m / EarthRadiusM;
  const double latr = *lat_deg * DegToRad;
  const double lonr = *lon_deg * DegToRad;
  const double lat1sin = sin(latr);
  const double lat1cos = cos(latr);
  const double distcos = cos(offset);
  const double distsin = sin(offset);
  const double heading_r = heading_deg * DegToRad;

  const double new_lat = asin(lat1sin * distcos
    + lat1cos * distsin * cos(heading_r));
  const double new_lon = lonr + atan2(
    sin(heading_r) * distsin * lat1cos,
    distcos - lat1sin * sin(new_lat));

  *lat_deg = new_lat * RadToDeg;
  *lon_deg = new_lon * RadToDeg;
}

double SensorFusor::HorizontalMeasVarianceM2
  (
  const FusionGnssFix *fix
  ) const
{
  double sigma_m = 1.0;
  if (fix->rtk == GNSS_RTK_FIX)
  {
    sigma_m = 0.05;
  }
  else if (fix->rtk == GNSS_RTK_FLOAT)
  {
    sigma_m = 0.12;
  }
  else if (fix->fix_quality == 0)
  {
    sigma_m = 5.0;
  }
  else if (fix->fix_quality == 1)
  {
    sigma_m = 1.0;
  }
  else if (fix->fix_quality == 2)
  {
    sigma_m = 0.5;
  }
  else
  {
    sigma_m = 1.5;
  }

  if (fix->hdop > 0.0)
  {
    sigma_m *= (0.5 + 0.5 * fix->hdop);
  }
  if (sigma_m < HorizKfMinMeasSigmaM)
  {
    sigma_m = HorizKfMinMeasSigmaM;
  }
  return sigma_m * sigma_m;
}

void SensorFusor::GeodeticDeltaToNeM
  (
  double lat0_deg,
  double lon0_deg,
  double lat_deg,
  double lon_deg,
  double *north_m,
  double *east_m
  ) const
{
  const double dlat_rad = (lat_deg - lat0_deg) * DegToRad;
  const double dlon_rad = (lon_deg - lon0_deg) * DegToRad;
  const double lat0_rad = lat0_deg * DegToRad;
  *north_m = dlat_rad * EarthRadiusM;
  *east_m = dlon_rad * EarthRadiusM * cos(lat0_rad);
}

void SensorFusor::NeMToGeodetic
  (
  double lat0_deg,
  double lon0_deg,
  double north_m,
  double east_m,
  double *lat_deg,
  double *lon_deg
  ) const
{
  const double lat0_rad = lat0_deg * DegToRad;
  *lat_deg = lat0_deg + (north_m / EarthRadiusM) * RadToDeg;
  *lon_deg = lon0_deg
    + (east_m / (EarthRadiusM * cos(lat0_rad))) * RadToDeg;
}

void SensorFusor::ApplyFusedHorizontalKalman
  (
  double *lat_deg,
  double *lon_deg,
  const FusionGnssFix *fix_meta,
  bool measurement_valid
  )
{
  if (!horiz_kf_init)
  {
    if (!measurement_valid)
    {
      return;
    }
    kf_origin_lat_deg = *lat_deg;
    kf_origin_lon_deg = *lon_deg;
    kf_n_m = 0.0;
    kf_e_m = 0.0;
    kf_P_nn = 25.0;
    kf_P_ne = 0.0;
    kf_P_en = 0.0;
    kf_P_ee = 25.0;
    kf_last_fix_ms = fix_meta->last_fix_time_ms;
    horiz_kf_init = true;
    return;
  }

  double dt = 0.1;
  if (fix_meta->last_fix_time_valid
    && kf_last_fix_ms > 0
    && fix_meta->last_fix_time_ms >= kf_last_fix_ms)
  {
    const double d =
      (double)(fix_meta->last_fix_time_ms - kf_last_fix_ms) / 1000.0;
    if (d > 1e-3 && d < 5.0)
    {
      dt = d;
    }
  }
  kf_last_fix_ms = fix_meta->last_fix_time_ms;

  double q_scale = 1.0;
  if (fix_meta->vector.speed_kph > HorizKfSpeedBlendKph)
  {
    q_scale = 1.0
      + (fix_meta->vector.speed_kph - HorizKfSpeedBlendKph) * 1.0;
  }
  const double q = HorizKfProcessNoiseM2PerS * dt * q_scale;
  kf_P_nn += q;
  kf_P_ee += q;

  if (!measurement_valid)
  {
    NeMToGeodetic(
      kf_origin_lat_deg,
      kf_origin_lon_deg,
      kf_n_m,
      kf_e_m,
      lat_deg,
      lon_deg);
    return;
  }

  double R = HorizontalMeasVarianceM2(fix_meta);
  if (fix_meta->vector.speed_kph < HorizKfSpeedBlendKph)
  {
    R *= 1.5
      + (HorizKfSpeedBlendKph - fix_meta->vector.speed_kph) * 0.15;
  }

  double north_m = 0.0;
  double east_m = 0.0;
  GeodeticDeltaToNeM(
    kf_origin_lat_deg,
    kf_origin_lon_deg,
    *lat_deg,
    *lon_deg,
    &north_m,
    &east_m);

  const double y_n = north_m - kf_n_m;
  const double y_e = east_m - kf_e_m;

  const double S_nn = kf_P_nn + R;
  const double S_ne = kf_P_ne;
  const double S_en = kf_P_en;
  const double S_ee = kf_P_ee + R;

  const double det = S_nn * S_ee - S_ne * S_en;
  if (fabs(det) < 1e-18)
  {
    NeMToGeodetic(
      kf_origin_lat_deg,
      kf_origin_lon_deg,
      kf_n_m,
      kf_e_m,
      lat_deg,
      lon_deg);
    return;
  }

  const double inv_det = 1.0 / det;
  const double Sinv_nn = S_ee * inv_det;
  const double Sinv_ne = -S_ne * inv_det;
  const double Sinv_en = -S_en * inv_det;
  const double Sinv_ee = S_nn * inv_det;

  const double K_nn = kf_P_nn * Sinv_nn + kf_P_ne * Sinv_en;
  const double K_ne = kf_P_nn * Sinv_ne + kf_P_ne * Sinv_ee;
  const double K_en = kf_P_en * Sinv_nn + kf_P_ee * Sinv_en;
  const double K_ee = kf_P_en * Sinv_ne + kf_P_ee * Sinv_ee;

  kf_n_m += K_nn * y_n + K_ne * y_e;
  kf_e_m += K_en * y_n + K_ee * y_e;

  const double P_nn_new =
    (1.0 - K_nn) * kf_P_nn - K_ne * kf_P_en;
  const double P_ne_new =
    (1.0 - K_nn) * kf_P_ne - K_ne * kf_P_ee;
  const double P_en_new =
    -K_en * kf_P_nn + (1.0 - K_ee) * kf_P_en;
  const double P_ee_new =
    -K_en * kf_P_ne + (1.0 - K_ee) * kf_P_ee;

  kf_P_nn = P_nn_new;
  kf_P_ne = P_ne_new;
  kf_P_en = P_en_new;
  kf_P_ee = P_ee_new;

  NeMToGeodetic(
    kf_origin_lat_deg,
    kf_origin_lon_deg,
    kf_n_m,
    kf_e_m,
    lat_deg,
    lon_deg);
}

double SensorFusor::VerticalMeasVarianceM2
  (
  const FusionGnssFix *fix
  ) const
{
  double sigma_m = 2.0;
  if (fix->rtk == GNSS_RTK_FIX)
  {
    sigma_m = 0.06;
  }
  else if (fix->rtk == GNSS_RTK_FLOAT)
  {
    sigma_m = 0.16;
  }
  else if (fix->fix_quality == 0)
  {
    sigma_m = 10.0;
  }
  else if (fix->fix_quality == 1)
  {
    sigma_m = 2.0;
  }
  else if (fix->fix_quality == 2)
  {
    sigma_m = 1.0;
  }
  else
  {
    sigma_m = 3.0;
  }

  if (fix->hdop > 0.0)
  {
    sigma_m *= (0.5 + 0.5 * fix->hdop);
  }
  if (sigma_m < AltKfMinMeasSigmaM)
  {
    sigma_m = AltKfMinMeasSigmaM;
  }
  return sigma_m * sigma_m;
}

void SensorFusor::ApplyFusedAltitudeKalman
  (
  double *altitude_m,
  const FusionGnssFix *fix_meta
  )
{
  if (!alt_kf_init)
  {
    kf_alt_m = *altitude_m;
    kf_P_alt = 25.0;
    kf_alt_last_fix_ms = fix_meta->last_fix_time_ms;
    alt_kf_init = true;
    return;
  }

  double dt = 0.1;
  if (fix_meta->last_fix_time_valid
    && kf_alt_last_fix_ms > 0
    && fix_meta->last_fix_time_ms >= kf_alt_last_fix_ms)
  {
    const double d =
      (double)(fix_meta->last_fix_time_ms - kf_alt_last_fix_ms) / 1000.0;
    if (d > 1e-3 && d < 5.0)
    {
      dt = d;
    }
  }
  kf_alt_last_fix_ms = fix_meta->last_fix_time_ms;

  double q_scale = 1.0;
  if (fix_meta->vector.speed_kph > HorizKfSpeedBlendKph)
  {
    q_scale = 1.0
      + (fix_meta->vector.speed_kph - HorizKfSpeedBlendKph) * 1.0;
  }
  const double q = AltKfProcessNoiseM2PerS * dt * q_scale;
  kf_P_alt += q;

  const double R = VerticalMeasVarianceM2(fix_meta);
  const double y = *altitude_m - kf_alt_m;
  const double S = kf_P_alt + R;
  const double K = kf_P_alt / S;

  kf_alt_m += K * y;
  kf_P_alt *= (1.0 - K);

  *altitude_m = kf_alt_m;
}

// core fusion: heading blend, tilt and antenna lever-arm, updates last_* and fix_out
void SensorFusor::FuseInternal
  (
  const FusionGnssFix *fix_in,           // input GNSS fix and metadata
  const FusionImuValue *imu,             // IMU attitude and rates
  uint32_t antenna_height_mm,            // antenna height above IMU (mm)
  int32_t antenna_left_mm,               // antenna left offset, + = port (mm)
  int32_t antenna_forward_mm,            // antenna forward offset, + = ahead (mm)
  FusionGnssFix *fix_out                 // output corrected fix
  )
{
  const double antenna_height_m  = (double)antenna_height_mm / 1000.0;
  const double antenna_left_m    = (double)antenna_left_mm / 1000.0;
  const double antenna_forward_m = (double)antenna_forward_mm / 1000.0;

  double gyro_heading = 0.0;
  const double imu_yaw_rate = imu->yaw_rate;
  double heading = InvalidHeading;

  const bool yaw_rate_ok_for_horiz_lever =
    fabs(imu_yaw_rate) < YawRateThreshold;

  const double imu_heading = imu->heading + MagneticDeclinationDeg;
  double latitude = fix_in->latitude;
  double longitude = fix_in->longitude;
  double altitude = fix_in->altitude;

  const double fuse_in_lat = fix_in->latitude;
  const double fuse_in_lon = fix_in->longitude;
  const double fuse_in_alt = fix_in->altitude;

  // Base heading is always IMU + magnetic declination.
  // When moving at speed with RTK, blend with GNSS COG for better accuracy.
  heading = NormalizeHeadingDeg(imu_heading);

  if (imu_gyro_offset == InvalidGyro)
  {
    if (fix_in->vector.speed_kph > SpeedThresholdKph
      && FixHasRtk(fix_in))
    {
      heading = fix_in->vector.track_magnetic_deg;
      imu_gyro_offset = heading - imu_heading;
    }
  }
  else
  {
    gyro_heading = imu_heading + imu_gyro_offset;

    if (gyro_heading < 0.0)
    {
      gyro_heading += 360.0;
    }
    else if (gyro_heading >= 360.0)
    {
      gyro_heading -= 360.0;
    }

    if (FixHasRtk(fix_in)
      && fix_in->vector.speed_kph > SpeedThresholdKph
      && fabs(imu_yaw_rate) < YawRateThreshold)
    {
      heading = fix_in->vector.track_magnetic_deg * 0.6
        + gyro_heading * 0.4;
      imu_gyro_offset = heading - imu_heading;
    }
    else
    {
      heading = gyro_heading;
    }
  }

  if (heading != InvalidHeading
    && (imu->roll != 0.0 || imu->pitch != 0.0)
    && FixHasRtk(fix_in))
  {
    double heading90 = heading + 90.0;
    if (heading90 >= 360.0)
    {
      heading90 -= 360.0;
    }

    double center_offset = 0.0;
    double alt_offset2 = 0.0;

    if (antenna_left_m != 0.0)
    {
      center_offset = cos(imu->roll * DegToRad) * antenna_left_m;
      alt_offset2 = sin(imu->roll * DegToRad) * center_offset;
    }

    const double roll_tilt_offset =
      -sin(imu->roll * DegToRad) * antenna_height_m;
    const double pitch_tilt_offset =
      sin(imu->pitch * DegToRad) * antenna_height_m;
    const double alt_offset1 =
      cos(imu->roll * DegToRad)
      * cos(imu->pitch * DegToRad)
      * antenna_height_m;

    FusionGnssFix corrected;
    corrected.latitude            = latitude;
    corrected.longitude           = longitude;
    corrected.vector              = fix_in->vector;
    corrected.fix_quality         = fix_in->fix_quality;
    corrected.hdop                = fix_in->hdop;
    corrected.rtk                 = fix_in->rtk;
    corrected.last_fix_time_valid = 0;
    corrected.last_fix_time_ms    = 0;

    if (yaw_rate_ok_for_horiz_lever && imu->roll != 0.0)
    {
      MoveDistanceBearing(
        &corrected.latitude,
        &corrected.longitude,
        heading90,
        roll_tilt_offset + center_offset);
    }

    if (yaw_rate_ok_for_horiz_lever && imu->pitch != 0.0)
    {
      MoveDistanceBearing(
        &corrected.latitude,
        &corrected.longitude,
        heading,
        pitch_tilt_offset);
    }

    altitude -= (alt_offset1 - alt_offset2);

    if (yaw_rate_ok_for_horiz_lever && antenna_forward_m != 0.0)
    {
      MoveDistanceBearing(
        &corrected.latitude,
        &corrected.longitude,
        heading,
        -antenna_forward_m);
    }

    corrected.altitude = altitude;

    last_latitude  = latitude;
    last_longitude = longitude;

    ApplyFusedAltitudeKalman(&corrected.altitude, fix_in);
    ApplyFusedHorizontalKalman(&corrected.latitude, &corrected.longitude, fix_in, yaw_rate_ok_for_horiz_lever);

    *fix_out = corrected;

    ConvertResultstoMm(fuse_in_lat, fuse_in_lon, corrected.latitude, corrected.longitude, corrected.altitude - fuse_in_alt);
    return;
  }

  if (heading != InvalidHeading && FixHasRtk(fix_in))
  {
    double heading90 = heading + 90.0;
    if (heading90 >= 360.0)
    {
      heading90 -= 360.0;
    }

    double center_offset = 0.0;
    double alt_offset2 = 0.0;

    if (antenna_left_m != 0.0)
    {
      center_offset = cos(imu->roll * DegToRad) * antenna_left_m;
      alt_offset2 = sin(imu->roll * DegToRad) * center_offset;
    }

    const double alt_offset1 = antenna_height_m;

    FusionGnssFix corrected;
    corrected.latitude            = latitude;
    corrected.longitude           = longitude;
    corrected.vector              = fix_in->vector;
    corrected.fix_quality         = fix_in->fix_quality;
    corrected.hdop                = fix_in->hdop;
    corrected.rtk                 = fix_in->rtk;
    corrected.last_fix_time_valid = fix_in->last_fix_time_valid;
    corrected.last_fix_time_ms    = fix_in->last_fix_time_ms;

    altitude -= (alt_offset1 - alt_offset2);

    if (yaw_rate_ok_for_horiz_lever && antenna_forward_m != 0.0)
    {
      MoveDistanceBearing(
        &corrected.latitude,
        &corrected.longitude,
        heading,
        -antenna_forward_m);
    }

    corrected.altitude = altitude;

    last_latitude  = latitude;
    last_longitude = longitude;

    ApplyFusedAltitudeKalman(&corrected.altitude, fix_in);
    ApplyFusedHorizontalKalman(&corrected.latitude, &corrected.longitude, fix_in, yaw_rate_ok_for_horiz_lever);

    *fix_out = corrected;

    ConvertResultstoMm(fuse_in_lat, fuse_in_lon, corrected.latitude, corrected.longitude, corrected.altitude - fuse_in_alt);

    return;
  }

  {
    double heading90 = heading + 90.0;
    if (heading90 >= 360.0)
    {
      heading90 -= 360.0;
    }

    double center_offset = 0.0;
    double alt_offset2 = 0.0;

    if (antenna_left_m != 0.0)
    {
      center_offset = cos(imu->roll * DegToRad) * antenna_left_m;
      alt_offset2 = sin(imu->roll * DegToRad) * center_offset;
    }

    const double roll_tilt_offset =
      -sin(imu->roll * DegToRad) * antenna_height_m;
    const double pitch_tilt_offset =
      sin(imu->pitch * DegToRad) * antenna_height_m;
    const double alt_offset1 =
      cos(imu->roll * DegToRad)
      * cos(imu->pitch * DegToRad)
      * antenna_height_m;

    altitude -= (alt_offset1 - alt_offset2);

    FusionGnssFix corrected;
    corrected.latitude            = latitude;
    corrected.longitude           = longitude;
    corrected.vector              = fix_in->vector;
    corrected.fix_quality         = fix_in->fix_quality;
    corrected.hdop                = fix_in->hdop;
    corrected.rtk                 = fix_in->rtk;
    corrected.last_fix_time_valid = fix_in->last_fix_time_valid;
    corrected.last_fix_time_ms    = fix_in->last_fix_time_ms;

    if (imu->roll != 0.0)
    {
      MoveDistanceBearing(
        &corrected.latitude,
        &corrected.longitude,
        heading90,
        roll_tilt_offset + center_offset);
    }

    if (imu->pitch != 0.0)
    {
      MoveDistanceBearing(
        &corrected.latitude,
        &corrected.longitude,
        heading,
        pitch_tilt_offset);
    }

    if (antenna_forward_m != 0.0)
    {
      MoveDistanceBearing(
        &corrected.latitude,
        &corrected.longitude,
        heading,
        -antenna_forward_m);
    }

    corrected.altitude = altitude;

    last_latitude  = latitude;
    last_longitude = longitude;

    alt_kf_init   = false;
    horiz_kf_init = false;

    *fix_out = corrected;

    ConvertResultstoMm(fuse_in_lat, fuse_in_lon, corrected.latitude, corrected.longitude, corrected.altitude - fuse_in_alt);
  }
}

///////////////////////////////////////////////////////////////////////////////////
// PUBLIC FUNCTIONS

// constructor; initializes fusion state (no prior fix for low-speed heading)
SensorFusor::SensorFusor
  (
  void
  )
  : last_latitude(InvalidLatitude)
  , last_longitude(InvalidLongitude)
{
  FuseAppliedCallback = NULL;
}

// combines GNSS fix and IMU into a corrected gnss_location_t
void SensorFusor::Fuse
  (
  gnss_location_t *plocation,           // GNSS fix, speed, RTK, antenna offsets (mm)
  const IMU &ImuHandler,                // IMU handler (for timestamped buffer lookup)
  uint8_t ImuIndex,                     // which IMU (TRACTOR_IDX, FRONT_BLADE_IDX, etc.)
  const antenna_location_t &Antenna     // location of antenna 
  )
{
  FusionGnssFix fix_in;
  fix_in.latitude                  = plocation->Latitude;
  fix_in.longitude                 = plocation->Longitude;
  fix_in.altitude                  = plocation->Altitude;
  fix_in.vector.track_magnetic_deg = plocation->TrackMagneticDeg;
  fix_in.vector.speed_kph          = plocation->SpeedKph;
  fix_in.fix_quality               = plocation->FixQuality;
  fix_in.hdop                      = plocation->Hdop;
  fix_in.rtk                       = plocation->RtkStatus;
  fix_in.last_fix_time_valid       = plocation->LastFixTimeValid;
  fix_in.last_fix_time_ms          = plocation->LastFixTimeMs;

  imu_t imu;
  if (!ImuHandler.GetSampleAtTime(ImuIndex, plocation->FixEpochMs, &imu))
  {
    imu = ImuHandler.IMUValues[ImuIndex];
  }

  FusionImuValue imu_val;
  imu_val.pitch    = (double)imu.Pitch - IMU_PITCH_CALIBRATION_DEG;
  imu_val.heading  = (double)imu.Heading;
  imu_val.roll     = (double)imu.Roll - IMU_ROLL_CALIBRATION_DEG;
  imu_val.yaw_rate = (double)imu.YawRate;

  FusionGnssFix fix_out;
  FuseInternal(
    &fix_in,
    &imu_val,
    Antenna.HeightMm,
    Antenna.LeftMm,
    Antenna.ForwardMm,
    &fix_out);

  plocation->Latitude         = fix_out.latitude;
  plocation->Longitude        = fix_out.longitude;
  plocation->Altitude         = fix_out.altitude;
  plocation->TrackMagneticDeg = fix_out.vector.track_magnetic_deg;
  plocation->SpeedKph         = fix_out.vector.speed_kph;
  plocation->RtkStatus        = fix_out.rtk;
  plocation->LastFixTimeValid = fix_out.last_fix_time_valid;
  plocation->LastFixTimeMs    = fix_out.last_fix_time_ms;
}

// sets the callback functions
void SensorFusor::SetCallbacks
  ( 
  sensorfusor_fuse_applied_t _FuseAppliedCallback
  )
{
  FuseAppliedCallback = _FuseAppliedCallback;
}
