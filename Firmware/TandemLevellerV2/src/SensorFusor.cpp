// GNSS / IMU fusion

#include <math.h>
#include "SensorFusor.h"

///////////////////////////////////////////////////////////////////////////////////
// PRIVATE FUNCTIONS

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

  const double imu_heading = imu->heading;
  double latitude = fix_in->latitude;
  double longitude = fix_in->longitude;
  double altitude = fix_in->altitude;

  // we haven't calculated the offset between the GNSS heading and the IMU heading yet
  if (imu_gyro_offset == InvalidGyro)
  {
    if (fix_in->vector.speed_kph > SpeedThresholdKph
      && FixHasRtk(fix_in))
    {
      heading = fix_in->vector.track_magnetic_deg;
    }
    else
    {
      if (last_latitude == InvalidLatitude)
      {
        last_latitude = latitude;
        last_longitude = longitude;
      }
      else if (HaversineDistanceM(
                 last_latitude,
                 last_longitude,
                 latitude,
                 longitude)
        > FixHeadingMinM)
      {
        heading = BearingDegrees(
          last_latitude,
          last_longitude,
          latitude,
          longitude);
      }

      if (heading != InvalidHeading)
      {
        imu_gyro_offset = heading - imu_heading;
      }
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
      && imu_yaw_rate < YawRateThreshold)
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
      sin(imu->roll * DegToRad) * antenna_height_m;
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
    corrected.rtk                 = fix_in->rtk;
    corrected.last_fix_time_valid = 0;
    corrected.last_fix_time_ms    = 0;

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

    altitude -= (alt_offset1 - alt_offset2);

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

    *fix_out = corrected;
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
    corrected.rtk                 = fix_in->rtk;
    corrected.last_fix_time_valid = fix_in->last_fix_time_valid;
    corrected.last_fix_time_ms    = fix_in->last_fix_time_ms;

    altitude -= (alt_offset1 - alt_offset2);

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

    *fix_out = corrected;
    return;
  }

  {
    FusionGnssFix corrected;
    corrected.latitude            = fix_in->latitude;
    corrected.longitude           = fix_in->longitude;
    corrected.altitude            = altitude;
    corrected.vector              = fix_in->vector;
    corrected.rtk                 = fix_in->rtk;
    corrected.last_fix_time_valid = fix_in->last_fix_time_valid;
    corrected.last_fix_time_ms    = fix_in->last_fix_time_ms;

    last_latitude  = fix_in->latitude;
    last_longitude = fix_in->longitude;

    *fix_out = corrected;
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
}

// combines GNSS fix and IMU into a corrected gnss_location_t
void SensorFusor::Fuse
  (
  gnss_location_t *plocation,           // GNSS fix, speed, RTK, antenna offsets (mm)
  const imu_t &imu,                     // roll, pitch, heading, yaw rate
  const antenna_location_t &Antenna     // location of antenna 
  )
{
  FusionGnssFix fix_in;
  fix_in.latitude                  = plocation->Latitude;
  fix_in.longitude                 = plocation->Longitude;
  fix_in.altitude                  = plocation->Altitude;
  fix_in.vector.track_magnetic_deg = plocation->TrackMagneticDeg;
  fix_in.vector.speed_kph          = plocation->SpeedKph;
  fix_in.rtk                       = plocation->RtkStatus;
  fix_in.last_fix_time_valid       = plocation->LastFixTimeValid;
  fix_in.last_fix_time_ms          = plocation->LastFixTimeMs;

  FusionImuValue imu_val;
  imu_val.pitch    = (double)imu.Pitch;
  imu_val.heading  = (double)imu.Heading;
  imu_val.roll     = (double)imu.Roll;
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
