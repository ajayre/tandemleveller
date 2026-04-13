// GNSS / IMU fusion (port of SensorFusor.cs)

#ifndef _SENSORFUSORH_
#define _SENSORFUSORH_

#include <Arduino.h>
#include "Global.h"
#include "GNSS.h"
#include "IMU.h"

class SensorFusor;

// fuse applied callback
typedef void (*sensorfusor_fuse_applied_t)(SensorFusor *pFusor, int EastingMm, int NorthingMm, int AltitudeMm);

class SensorFusor
{
  public:
    // constructor; initializes fusion state (no prior fix for low-speed heading)
    SensorFusor
      (
      void
      );

    // combines GNSS fix and IMU attitude into a lever-arm / tilt corrected location
    void Fuse
      (
      gnss_location_t *plocation,           // GNSS fix, speed, RTK, antenna offsets (mm)
      const imu_t &imu,                     // roll, pitch, heading, yaw rate
      const antenna_location_t &Antenna     // location of antenna 
      );

    // sets the callback functions
    void SetCallbacks
      ( 
      sensorfusor_fuse_applied_t FuseAppliedCallback
      );

  private:
    double imu_gyro_offset = InvalidGyro;
    double last_latitude;
    double last_longitude;
    sensorfusor_fuse_applied_t FuseAppliedCallback;

    struct FusionGnssVector
    {
      double track_magnetic_deg;
      double speed_kph;
    };

    struct FusionGnssFix
    {
      double latitude;
      double longitude;
      double altitude;
      FusionGnssVector vector;
      gnss_rtk_status_t rtk;
      int last_fix_time_valid;
      uint32_t last_fix_time_ms;
    };

    struct FusionImuValue
    {
      double pitch;
      double heading;
      double roll;
      double yaw_rate;
    };

    static constexpr double SpeedThresholdKph    = 3.0;
    static constexpr double FixHeadingMinM       = 1.0;
    static constexpr double YawRateThreshold     = 6.0;

    static constexpr double EarthRadiusM           = 6378137.0;
    static constexpr double Pi                     = 3.14159265358979323846;
    static constexpr double DegToRad               = (Pi / 180.0);
    static constexpr double RadToDeg               = (180.0 / Pi);

    static constexpr double InvalidLatitude        = -301.0;
    static constexpr double InvalidLongitude       = -301.0;
    static constexpr double InvalidHeading         = -301.0;
    static constexpr double InvalidGyro            = -401.0;

    // returns non-zero if fix has RTK fixed or float solution
    int FixHasRtk
      (
      const FusionGnssFix *fix               // fix to test for RTK fixed or float
      ) const;

    // great-circle distance in meters between two WGS84 lat/lon points (degrees)
    double HaversineDistanceM
      (
      double from_lat_deg,                   // start latitude (degrees)
      double from_lon_deg,                   // start longitude (degrees)
      double to_lat_deg,                     // end latitude (degrees)
      double to_lon_deg                      // end longitude (degrees)
      ) const;

    // initial bearing from first point to second, radians, -pi..pi
    double BearingRad
      (
      double from_lat_deg,                   // start latitude (degrees)
      double from_lon_deg,                   // start longitude (degrees)
      double to_lat_deg,                     // end latitude (degrees)
      double to_lon_deg                      // end longitude (degrees)
      ) const;

    // initial bearing in degrees (matches legacy BearingDegrees lon/lat argument order)
    double BearingDegrees
      (
      double from_lat_deg,                   // start latitude (degrees)
      double from_lon_deg,                   // start longitude (degrees)
      double to_lat_deg,                     // end latitude (degrees)
      double to_lon_deg                      // end longitude (degrees)
      ) const;

    // heading in degrees -> [0, 360)
    double NormalizeHeadingDeg
      (
      double heading_deg
      ) const;

    // moves (*lat_deg,*lon_deg) by distance_m along heading_deg (degrees, clockwise from north)
    void MoveDistanceBearing
      (
      double *lat_deg,                       // in-out latitude (degrees)
      double *lon_deg,                       // in-out longitude (degrees)
      double heading_deg,                    // bearing clockwise from north (degrees)
      double distance_m                      // distance to move (meters)
      ) const;

    // core fusion: updates last_latitude/longitude and writes corrected fix_out
    void FuseInternal
      (
      const FusionGnssFix *fix_in,           // input GNSS fix and metadata
      const FusionImuValue *imu,             // IMU attitude and rates
      uint32_t antenna_height_mm,            // antenna height above IMU (mm)
      int32_t antenna_left_mm,               // antenna left offset, + = port (mm)
      int32_t antenna_forward_mm,            // antenna forward offset, + = ahead (mm)
      FusionGnssFix *fix_out                 // output corrected fix
      );

    // Local tangent plane: North/East displacement (mm, fused minus input) and altitude
    // delta in meters (fused minus input); callback Up mm = delta_alt_m * 1000.
    void ConvertResultstoMm
      (
      double in_lat_deg,
      double in_lon_deg,
      double out_lat_deg,
      double out_lon_deg,
      double delta_alt_m
      );
};

#endif // _SENSORFUSORH_
