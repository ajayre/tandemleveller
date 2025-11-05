using Controller_Test_Harness;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Controller
{
    // Adapted from: https://github.com/torriem/f9p_imu/blob/master/f9p_imu.ino
    public class SensorFusor
    {
        private const int SPEED_THRESHOLD    = 3;  // kph
        private const int FIX_HEADING_MIN    = 1;  // meters
        private const int YAW_RATE_THRESHOLD = 6;  // deg/sec

        private double LastLatitude = -301;
        private double LastLongitude = -301;

        /// <summary>
        /// Fuses the GNSS and IMU sensor readings
        /// </summary>
        /// <param name="Fix">GNSS location and height</param>
        /// <param name="IMUReading">IMU reading</param>
        /// <param name="AntennaHeight">Height of antenna in meters</param>
        /// <returns>Corrected GNSS location and height</returns>
        public GNSSFix Fuse
            (
            GNSSFix Fix,
            IMUValue IMUReading,
            double AntennaHeight
            )
        {
            double GyroHeading = 0;
            double IMUYawRate = IMUReading.YawRate;

            double Heading = -301;

            // assume started pointing straight north
            double IMUGyroOffset = -401;

            // get heading from IMU
            double IMUHeading = IMUReading.Heading;

            double Latitude = Fix.Latitude;
            double Longitude = Fix.Longitude;

            double Altitude = Fix.Altitude;

            // no gyro offset is known
            if (IMUGyroOffset < -400)
            {
                // we are going fast enough to use GNSS heading
                if (Fix.Speed > SPEED_THRESHOLD && Fix.HasRTK)
                {
                    Heading = Fix.Heading;
                }
                // assume going forward, but it's still slow, so we need to go at least some distance
                else
                {
                    // we don't have a previous fix so get the current location
                    if (LastLatitude < -300)
                    {
                        LastLatitude = Latitude;
                        LastLongitude = Longitude;
                    }
                    // we have moved far enough to measure
                    else if (Haversine.Distance(LastLatitude, LastLongitude, Latitude, Longitude) > FIX_HEADING_MIN)
                    {
                        // get a heading from the movement
                        Heading = Haversine.BearingDegrees(LastLatitude, LastLongitude, Latitude, Longitude);
                    }

                    // we have a heading
                    if (Heading > -300)
                    {
                        IMUGyroOffset = Heading - IMUHeading;
                    }
                }
            }
            // we have a gyro offset we can use so we don't have to calculate using GNSS heading
            else
            {
                GyroHeading = IMUHeading + IMUGyroOffset;

                // wrap around
                if (GyroHeading < 0) GyroHeading += 360;
                else if (GyroHeading >= 360) GyroHeading -= 360;

                // assume we're going forward, at a decent speed, and not turning very fast
                // we'll use mostly gps heading with some gyro heading (adjust?)
                if (Fix.HasRTK && Fix.Speed > SPEED_THRESHOLD && IMUYawRate < YAW_RATE_THRESHOLD)
                {
                    Heading = Fix.Heading * 0.6 + GyroHeading * 0.4;

                    // recompute offset
                    IMUGyroOffset = Heading - IMUHeading;
                }
                // we're either going really slow, or turning very fast so rely on gyro with offset
                else
                {
                    Heading = GyroHeading;
                }
            }

            // if we have a heading and we are rolling and/or pitching and have RTK then compute
            if ((Heading > -300) && ((IMUReading.Roll != 0) || (IMUReading.Pitch != 0)) && Fix.HasRTK)
            {
                // use the imu roll to do terrain compensation: adjust lat, lon and altitude

                // rotate the heading 90 degrees to give the direction of roll
                double Heading90 = Heading + 90;
                if (Heading90 >= 360) Heading90 -= 360;

                // Calculate horizontal displacements for roll (perpendicular to heading)
                double RollTiltOffset = Math.Sin(IMUReading.Roll * Math.PI / 180.0) * AntennaHeight;

                // Calculate horizontal displacements for pitch (along heading)
                double PitchTiltOffset = Math.Sin(IMUReading.Pitch * Math.PI / 180.0) * AntennaHeight;

                // Calculate vertical height above ground accounting for both roll and pitch
                // When both angles are present, the vertical component is: cos(roll) × cos(pitch) × AntennaHeight
                double AltOffset1 = Math.Cos(IMUReading.Roll * Math.PI / 180.0) *
                                    Math.Cos(IMUReading.Pitch * Math.PI / 180.0) *
                                    AntennaHeight;

                GNSSFix CorrectedFix = new GNSSFix();
                CorrectedFix.Latitude = Latitude;
                CorrectedFix.Longitude = Longitude;

                // Correct position for roll displacement (perpendicular to heading)
                if (IMUReading.Roll != 0)
                {
                    Haversine.MoveDistanceBearing(ref CorrectedFix.Latitude, ref CorrectedFix.Longitude, Heading90, RollTiltOffset);
                }

                // Correct position for pitch displacement (along heading)
                if (IMUReading.Pitch != 0)
                {
                    Haversine.MoveDistanceBearing(ref CorrectedFix.Latitude, ref CorrectedFix.Longitude, Heading, PitchTiltOffset);
                }

                // we subtract the new antenna height above ground from the antenna height to get the corrected ground height
                // note: when roll = 0, AltOffset1 will be the vehicle height, which means no correction and the full vehicle height is subtracted
                Altitude -= AltOffset1;

                CorrectedFix.Altitude = Altitude;

                CorrectedFix.HasRTK = Fix.HasRTK;
                CorrectedFix.Heading = Fix.Heading;
                CorrectedFix.Speed = Fix.Speed;

                LastLatitude = Latitude;
                LastLongitude = Longitude;

                return CorrectedFix;
            }
            // no change
            else
            {
                LastLatitude = Fix.Latitude;
                LastLongitude = Fix.Longitude;

                return Fix;
            }
        }
    }
}
