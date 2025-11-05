using Controller_Test_Harness;
using Controller;
using System;

namespace Controller_Test_Harness
{
    public class SensorFusor_Tests
    {
        private const double ANTENNA_HEIGHT = 4.2672; // 14 feet in meters
        private const double POSITION_TOLERANCE = 0.0001; // meters (for lat/lon differences)
        private const double ALTITUDE_TOLERANCE = 0.01; // meters
        private const double HEADING_TOLERANCE = 0.1; // degrees

        /// <summary>
        /// Creates a GNSSFix from degrees (latitude/longitude) and meters (altitude)
        /// </summary>
        private GNSSFix CreateGNSSFix(double longitudeDeg, double latitudeDeg, double altitudeM, 
                                       double heading, double speedKph, bool hasRTK)
        {
            return new GNSSFix(
                longitudeDeg,
                latitudeDeg,
                altitudeM,
                heading,
                speedKph,
                hasRTK
            );
        }

        /// <summary>
        /// Runs all tests and returns true if all pass, false if any fail
        /// </summary>
        public bool Run()
        {
            bool allPassed = true;

            Console.WriteLine("Running SensorFusor Tests...");
            Console.WriteLine("============================\n");

            allPassed &= TestNoCorrection_NoRTK();
            allPassed &= TestNoCorrection_NoHeading();
            allPassed &= TestNoCorrection_ZeroRoll();
            allPassed &= TestRoll_15Degrees_Right();
            allPassed &= TestRoll_15Degrees_Left();
            allPassed &= TestRoll_30Degrees_Right();
            allPassed &= TestRoll_45Degrees_Right();
            allPassed &= TestRoll_90Degrees_Right();
            allPassed &= TestRoll_1Degree_Right();
            allPassed &= TestRoll_HeadingEast();
            allPassed &= TestRoll_HeadingNortheast();
            allPassed &= TestRoll_HeadingWrapAround();
            allPassed &= TestPitch_15Degrees_Forward();
            allPassed &= TestRollAndPitch_Combined();
            allPassed &= TestRollAndPitch_Moderate();
            allPassed &= TestRollAndPitch_LargePitchSmallRoll();
            allPassed &= TestRollAndPitch_SmallPitchLargeRoll();
            allPassed &= TestRollAndPitch_NegativePitchPositiveRoll();
            allPassed &= TestRollAndPitch_PositivePitchNegativeRoll();

            Console.WriteLine("\n============================");
            if (allPassed)
            {
                Console.WriteLine("All tests PASSED!");
            }
            else
            {
                Console.WriteLine("Some tests FAILED!");
            }

            return allPassed;
        }

        private bool TestNoCorrection_NoRTK()
        {
            Console.Write("Test 1: No Correction (No RTK)... ");

            SensorFusor fusor = new SensorFusor();
            GNSSFix input = CreateGNSSFix(
                -100.0,  // longitude degrees
                40.0,    // latitude degrees
                100.0,   // altitude meters
                0.0,     // heading
                5.0,     // speed kph
                false    // No RTK
            );

            IMUValue imu = new IMUValue
            {
                Pitch = 5.0,
                Roll = 10.0,
                Yaw = 0.0,
                YawRate = 0.0,
                Heading = 0.0
            };

            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT);

            bool passed = AreEqual(input, result);
            Console.WriteLine(passed ? "PASS" : "FAIL");
            return passed;
        }

        private bool TestNoCorrection_NoHeading()
        {
            Console.Write("Test 2: No Correction (No Heading)... ");

            SensorFusor fusor = new SensorFusor();
            GNSSFix input = CreateGNSSFix(
                -100.0,  // longitude degrees
                40.0,    // latitude degrees
                100.0,   // altitude meters
                -301.0,  // Invalid heading
                5.0,     // speed kph
                true
            );

            IMUValue imu = new IMUValue
            {
                Pitch = 0.0,
                Roll = 10.0,
                Yaw = 0.0,
                YawRate = 0.0,
                Heading = 0.0
            };

            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT);

            bool passed = AreEqual(input, result);
            Console.WriteLine(passed ? "PASS" : "FAIL");
            return passed;
        }

        private bool TestNoCorrection_ZeroRoll()
        {
            Console.Write("Test 3: No Correction (Zero Roll)... ");

            SensorFusor fusor = new SensorFusor();
            GNSSFix input = CreateGNSSFix(
                -100.0,  // longitude degrees
                40.0,    // latitude degrees
                100.0,   // altitude meters
                0.0,     // heading
                5.0,     // speed kph
                true
            );

            IMUValue imu = new IMUValue
            {
                Pitch = 0.0,
                Roll = 0.0, // No roll
                Yaw = 0.0,
                YawRate = 0.0,
                Heading = 0.0
            };

            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT);

            bool passed = AreEqual(input, result);
            Console.WriteLine(passed ? "PASS" : "FAIL");
            return passed;
        }

        private bool TestRoll_15Degrees_Right()
        {
            Console.Write("Test 4: Roll 15° Right (North Heading)... ");

            SensorFusor fusor = new SensorFusor();
            GNSSFix input = CreateGNSSFix(
                -100.0,  // longitude degrees
                40.0,    // latitude degrees
                100.0,   // altitude meters
                0.0,     // heading (North)
                5.0,     // speed kph
                true
            );

            IMUValue imu = new IMUValue
            {
                Pitch = 0.0,
                Roll = 15.0,
                Yaw = 0.0,
                YawRate = 0.0,
                Heading = 0.0
            };

            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT);

            // Expected: Shifted ~1.1044m west (increased longitude), altitude -4.1218m
            double expectedRollOffset = Math.Sin(15.0 * Math.PI / 180.0) * ANTENNA_HEIGHT; // 1.1044m
            double expectedAltOffset = Math.Cos(15.0 * Math.PI / 180.0) * ANTENNA_HEIGHT; // 4.1218m

            // When heading north (0°), roll moves antenna east, so we shift west (longitude increases)
            // The actual shift direction depends on Haversine calculation, so we check altitude primarily
            double expectedAltitude = (100.0 - expectedAltOffset);

            bool passed = Math.Abs(result.Altitude - expectedAltitude) < ALTITUDE_TOLERANCE &&
                          Math.Abs(result.Latitude - input.Latitude) < POSITION_TOLERANCE &&
                          result.HasRTK == input.HasRTK &&
                          result.Heading == input.Heading &&
                          result.Speed == input.Speed;

            Console.WriteLine(passed ? "PASS" : "FAIL");
            return passed;
        }

        private bool TestRoll_15Degrees_Left()
        {
            Console.Write("Test 5: Roll 15° Left (North Heading)... ");

            SensorFusor fusor = new SensorFusor();
            GNSSFix input = CreateGNSSFix(
                -100.0,  // longitude degrees
                40.0,    // latitude degrees
                100.0,   // altitude meters
                0.0,     // heading (North)
                5.0,     // speed kph
                true
            );

            IMUValue imu = new IMUValue
            {
                Pitch = 0.0,
                Roll = -15.0, // Negative roll (left)
                Yaw = 0.0,
                YawRate = 0.0,
                Heading = 0.0
            };

            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT);

            double expectedAltOffset = Math.Cos(15.0 * Math.PI / 180.0) * ANTENNA_HEIGHT; // Same as positive
            double expectedAltitude = (100.0 - expectedAltOffset);

            bool passed = Math.Abs(result.Altitude - expectedAltitude) < ALTITUDE_TOLERANCE &&
                          result.HasRTK == input.HasRTK &&
                          result.Heading == input.Heading &&
                          result.Speed == input.Speed;

            Console.WriteLine(passed ? "PASS" : "FAIL");
            return passed;
        }

        private bool TestRoll_30Degrees_Right()
        {
            Console.Write("Test 6: Roll 30° Right (North Heading)... ");

            SensorFusor fusor = new SensorFusor();
            GNSSFix input = CreateGNSSFix(
                -100.0,  // longitude degrees
                40.0,    // latitude degrees
                100.0,   // altitude meters
                0.0,     // heading
                5.0,     // speed kph
                true
            );

            IMUValue imu = new IMUValue
            {
                Pitch = 0.0,
                Roll = 30.0,
                Yaw = 0.0,
                YawRate = 0.0,
                Heading = 0.0
            };

            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT);

            double expectedAltOffset = Math.Cos(30.0 * Math.PI / 180.0) * ANTENNA_HEIGHT; // 3.6955m
            double expectedAltitude = (100.0 - expectedAltOffset);

            bool passed = Math.Abs(result.Altitude - expectedAltitude) < ALTITUDE_TOLERANCE &&
                          result.HasRTK == input.HasRTK &&
                          result.Heading == input.Heading &&
                          result.Speed == input.Speed;

            Console.WriteLine(passed ? "PASS" : "FAIL");
            return passed;
        }

        private bool TestRoll_45Degrees_Right()
        {
            Console.Write("Test 7: Roll 45° Right (North Heading)... ");

            SensorFusor fusor = new SensorFusor();
            GNSSFix input = CreateGNSSFix(
                -100.0,  // longitude degrees
                40.0,    // latitude degrees
                100.0,   // altitude meters
                0.0,     // heading
                5.0,     // speed kph
                true
            );

            IMUValue imu = new IMUValue
            {
                Pitch = 0.0,
                Roll = 45.0,
                Yaw = 0.0,
                YawRate = 0.0,
                Heading = 0.0
            };

            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT);

            double expectedAltOffset = Math.Cos(45.0 * Math.PI / 180.0) * ANTENNA_HEIGHT; // 3.0174m
            double expectedAltitude = (100.0 - expectedAltOffset);

            bool passed = Math.Abs(result.Altitude - expectedAltitude) < ALTITUDE_TOLERANCE &&
                          result.HasRTK == input.HasRTK &&
                          result.Heading == input.Heading &&
                          result.Speed == input.Speed;

            Console.WriteLine(passed ? "PASS" : "FAIL");
            return passed;
        }

        private bool TestRoll_90Degrees_Right()
        {
            Console.Write("Test 8: Roll 90° Right (North Heading)... ");

            SensorFusor fusor = new SensorFusor();
            GNSSFix input = CreateGNSSFix(
                -100.0,  // longitude degrees
                40.0,    // latitude degrees
                100.0,   // altitude meters
                0.0,     // heading
                5.0,     // speed kph
                true
            );

            IMUValue imu = new IMUValue
            {
                Pitch = 0.0,
                Roll = 90.0,
                Yaw = 0.0,
                YawRate = 0.0,
                Heading = 0.0
            };

            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT);

            // At 90° roll, AltOffset1 should be 0 (antenna at ground level)
            double expectedAltitude = 100.0; // No altitude correction

            bool passed = Math.Abs(result.Altitude - expectedAltitude) < ALTITUDE_TOLERANCE &&
                          result.HasRTK == input.HasRTK &&
                          result.Heading == input.Heading &&
                          result.Speed == input.Speed;

            Console.WriteLine(passed ? "PASS" : "FAIL");
            return passed;
        }

        private bool TestRoll_1Degree_Right()
        {
            Console.Write("Test 9: Roll 1° Right (North Heading)... ");

            SensorFusor fusor = new SensorFusor();
            GNSSFix input = CreateGNSSFix(
                -100.0,  // longitude degrees
                40.0,    // latitude degrees
                100.0,   // altitude meters
                0.0,     // heading
                5.0,     // speed kph
                true
            );

            IMUValue imu = new IMUValue
            {
                Pitch = 0.0,
                Roll = 1.0,
                Yaw = 0.0,
                YawRate = 0.0,
                Heading = 0.0
            };

            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT);

            double expectedAltOffset = Math.Cos(1.0 * Math.PI / 180.0) * ANTENNA_HEIGHT; // ~4.2660m
            double expectedAltitude = (100.0 - expectedAltOffset);

            bool passed = Math.Abs(result.Altitude - expectedAltitude) < ALTITUDE_TOLERANCE &&
                          result.HasRTK == input.HasRTK &&
                          result.Heading == input.Heading &&
                          result.Speed == input.Speed;

            Console.WriteLine(passed ? "PASS" : "FAIL");
            return passed;
        }

        private bool TestRoll_HeadingEast()
        {
            Console.Write("Test 10: Roll 15° Right (East Heading)... ");

            SensorFusor fusor = new SensorFusor();
            GNSSFix input = CreateGNSSFix(
                -100.0,  // longitude degrees
                40.0,    // latitude degrees
                100.0,   // altitude meters
                90.0,    // heading (East)
                5.0,     // speed kph
                true
            );

            IMUValue imu = new IMUValue
            {
                Pitch = 0.0,
                Roll = 15.0,
                Yaw = 0.0,
                YawRate = 0.0,
                Heading = 90.0
            };

            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT);

            double expectedAltOffset = Math.Cos(15.0 * Math.PI / 180.0) * ANTENNA_HEIGHT;
            double expectedAltitude = (100.0 - expectedAltOffset);

            bool passed = Math.Abs(result.Altitude - expectedAltitude) < ALTITUDE_TOLERANCE &&
                          result.HasRTK == input.HasRTK &&
                          result.Heading == input.Heading &&
                          result.Speed == input.Speed;

            Console.WriteLine(passed ? "PASS" : "FAIL");
            return passed;
        }

        private bool TestRoll_HeadingNortheast()
        {
            Console.Write("Test 11: Roll 15° Right (Northeast Heading)... ");

            SensorFusor fusor = new SensorFusor();
            GNSSFix input = CreateGNSSFix(
                -100.0,  // longitude degrees
                40.0,    // latitude degrees
                100.0,   // altitude meters
                45.0,    // heading (Northeast)
                5.0,     // speed kph
                true
            );

            IMUValue imu = new IMUValue
            {
                Pitch = 0.0,
                Roll = 15.0,
                Yaw = 0.0,
                YawRate = 0.0,
                Heading = 45.0
            };

            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT);

            double expectedAltOffset = Math.Cos(15.0 * Math.PI / 180.0) * ANTENNA_HEIGHT;
            double expectedAltitude = (100.0 - expectedAltOffset);

            bool passed = Math.Abs(result.Altitude - expectedAltitude) < ALTITUDE_TOLERANCE &&
                          result.HasRTK == input.HasRTK &&
                          result.Heading == input.Heading &&
                          result.Speed == input.Speed;

            Console.WriteLine(passed ? "PASS" : "FAIL");
            return passed;
        }

        private bool TestRoll_HeadingWrapAround()
        {
            Console.Write("Test 12: Roll 15° Right (Heading 350°)... ");

            SensorFusor fusor = new SensorFusor();
            GNSSFix input = CreateGNSSFix(
                -100.0,  // longitude degrees
                40.0,    // latitude degrees
                100.0,   // altitude meters
                350.0,   // heading (Almost north - 10° west of north)
                5.0,     // speed kph
                true
            );

            IMUValue imu = new IMUValue
            {
                Pitch = 0.0,
                Roll = 15.0,
                Yaw = 0.0,
                YawRate = 0.0,
                Heading = 350.0
            };

            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT);

            // Heading90 = 350 + 90 = 440 → should wrap to 80°
            double expectedAltOffset = Math.Cos(15.0 * Math.PI / 180.0) * ANTENNA_HEIGHT;
            double expectedAltitude = (100.0 - expectedAltOffset);

            bool passed = Math.Abs(result.Altitude - expectedAltitude) < ALTITUDE_TOLERANCE &&
                          result.HasRTK == input.HasRTK &&
                          result.Heading == input.Heading &&
                          result.Speed == input.Speed;

            Console.WriteLine(passed ? "PASS" : "FAIL");
            return passed;
        }

        private bool TestPitch_15Degrees_Forward()
        {
            Console.Write("Test 13: Pitch 15° Forward (North Heading)... ");

            SensorFusor fusor = new SensorFusor();
            GNSSFix input = CreateGNSSFix(
                -100.0,  // longitude degrees
                40.0,    // latitude degrees
                100.0,   // altitude meters
                0.0,     // heading (North)
                5.0,     // speed kph
                true
            );

            IMUValue imu = new IMUValue
            {
                Pitch = 15.0, // Nose up (antenna moves forward)
                Roll = 0.0,
                Yaw = 0.0,
                YawRate = 0.0,
                Heading = 0.0
            };

            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT);

            // Pitch correction: AltOffset1 = cos(0°) × cos(15°) × AntennaHeight
            double expectedAltOffset = Math.Cos(0.0 * Math.PI / 180.0) * 
                                       Math.Cos(15.0 * Math.PI / 180.0) * 
                                       ANTENNA_HEIGHT;
            double expectedAltitude = (100.0 - expectedAltOffset);

            bool passed = Math.Abs(result.Altitude - expectedAltitude) < ALTITUDE_TOLERANCE &&
                          result.HasRTK == input.HasRTK &&
                          result.Heading == input.Heading &&
                          result.Speed == input.Speed;

            Console.WriteLine(passed ? "PASS" : "FAIL");
            return passed;
        }

        private bool TestRollAndPitch_Combined()
        {
            Console.Write("Test 14: Roll 15° + Pitch 15° Combined... ");

            SensorFusor fusor = new SensorFusor();
            GNSSFix input = CreateGNSSFix(
                -100.0,  // longitude degrees
                40.0,    // latitude degrees
                100.0,   // altitude meters
                0.0,     // heading (North)
                5.0,     // speed kph
                true
            );

            IMUValue imu = new IMUValue
            {
                Pitch = 15.0,
                Roll = 15.0,
                Yaw = 0.0,
                YawRate = 0.0,
                Heading = 0.0
            };

            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT);

            // Combined: AltOffset1 = cos(15°) × cos(15°) × AntennaHeight
            double expectedAltOffset = Math.Cos(15.0 * Math.PI / 180.0) * 
                                       Math.Cos(15.0 * Math.PI / 180.0) * 
                                       ANTENNA_HEIGHT; // ~3.9799m
            double expectedAltitude = (100.0 - expectedAltOffset);

            bool passed = Math.Abs(result.Altitude - expectedAltitude) < ALTITUDE_TOLERANCE &&
                          result.HasRTK == input.HasRTK &&
                          result.Heading == input.Heading &&
                          result.Speed == input.Speed;

            Console.WriteLine(passed ? "PASS" : "FAIL");
            return passed;
        }

        private bool TestRollAndPitch_Moderate()
        {
            Console.Write("Test 15: Roll 20° + Pitch 20° Combined... ");

            SensorFusor fusor = new SensorFusor();
            GNSSFix input = CreateGNSSFix(
                -100.0,  // longitude degrees
                40.0,    // latitude degrees
                100.0,   // altitude meters
                0.0,     // heading (North)
                5.0,     // speed kph
                true
            );

            IMUValue imu = new IMUValue
            {
                Pitch = 20.0,
                Roll = 20.0,
                Yaw = 0.0,
                YawRate = 0.0,
                Heading = 0.0
            };

            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT);

            // Combined: AltOffset1 = cos(20°) × cos(20°) × AntennaHeight
            double expectedAltOffset = Math.Cos(20.0 * Math.PI / 180.0) * 
                                       Math.Cos(20.0 * Math.PI / 180.0) * 
                                       ANTENNA_HEIGHT; // ~3.7686m
            double expectedAltitude = (100.0 - expectedAltOffset);

            bool passed = Math.Abs(result.Altitude - expectedAltitude) < ALTITUDE_TOLERANCE &&
                          result.HasRTK == input.HasRTK &&
                          result.Heading == input.Heading &&
                          result.Speed == input.Speed;

            Console.WriteLine(passed ? "PASS" : "FAIL");
            return passed;
        }

        private bool TestRollAndPitch_LargePitchSmallRoll()
        {
            Console.Write("Test 16: Roll 10° + Pitch 30° (Large Pitch)... ");

            SensorFusor fusor = new SensorFusor();
            GNSSFix input = CreateGNSSFix(
                -100.0,  // longitude degrees
                40.0,    // latitude degrees
                100.0,   // altitude meters
                0.0,     // heading (North)
                5.0,     // speed kph
                true
            );

            IMUValue imu = new IMUValue
            {
                Pitch = 30.0, // Large pitch (nose up)
                Roll = 10.0,  // Small roll
                Yaw = 0.0,
                YawRate = 0.0,
                Heading = 0.0
            };

            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT);

            // Combined: AltOffset1 = cos(10°) × cos(30°) × AntennaHeight
            double expectedAltOffset = Math.Cos(10.0 * Math.PI / 180.0) * 
                                       Math.Cos(30.0 * Math.PI / 180.0) * 
                                       ANTENNA_HEIGHT; // ~3.5726m
            double expectedAltitude = (100.0 - expectedAltOffset);

            // Pitch displacement: sin(30°) × 4.2672 = 2.1336m forward
            // Roll displacement: sin(10°) × 4.2672 = 0.7409m to the right

            bool passed = Math.Abs(result.Altitude - expectedAltitude) < ALTITUDE_TOLERANCE &&
                          result.HasRTK == input.HasRTK &&
                          result.Heading == input.Heading &&
                          result.Speed == input.Speed;

            Console.WriteLine(passed ? "PASS" : "FAIL");
            return passed;
        }

        private bool TestRollAndPitch_SmallPitchLargeRoll()
        {
            Console.Write("Test 17: Roll 30° + Pitch 10° (Large Roll)... ");

            SensorFusor fusor = new SensorFusor();
            GNSSFix input = CreateGNSSFix(
                -100.0,  // longitude degrees
                40.0,    // latitude degrees
                100.0,   // altitude meters
                0.0,     // heading (North)
                5.0,     // speed kph
                true
            );

            IMUValue imu = new IMUValue
            {
                Pitch = 10.0,  // Small pitch
                Roll = 30.0,   // Large roll
                Yaw = 0.0,
                YawRate = 0.0,
                Heading = 0.0
            };

            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT);

            // Combined: AltOffset1 = cos(30°) × cos(10°) × AntennaHeight
            double expectedAltOffset = Math.Cos(30.0 * Math.PI / 180.0) * 
                                       Math.Cos(10.0 * Math.PI / 180.0) * 
                                       ANTENNA_HEIGHT; // ~3.5726m
            double expectedAltitude = (100.0 - expectedAltOffset);

            // Roll displacement: sin(30°) × 4.2672 = 2.1336m to the right
            // Pitch displacement: sin(10°) × 4.2672 = 0.7409m forward

            bool passed = Math.Abs(result.Altitude - expectedAltitude) < ALTITUDE_TOLERANCE &&
                          result.HasRTK == input.HasRTK &&
                          result.Heading == input.Heading &&
                          result.Speed == input.Speed;

            Console.WriteLine(passed ? "PASS" : "FAIL");
            return passed;
        }

        private bool TestRollAndPitch_NegativePitchPositiveRoll()
        {
            Console.Write("Test 18: Roll 15° Right + Pitch -15° (Nose Down)... ");

            SensorFusor fusor = new SensorFusor();
            GNSSFix input = CreateGNSSFix(
                -100.0,  // longitude degrees
                40.0,    // latitude degrees
                100.0,   // altitude meters
                0.0,     // heading (North)
                5.0,     // speed kph
                true
            );

            IMUValue imu = new IMUValue
            {
                Pitch = -15.0, // Negative pitch (nose down, antenna moves backward)
                Roll = 15.0,    // Positive roll (rolling right, antenna moves right)
                Yaw = 0.0,
                YawRate = 0.0,
                Heading = 0.0
            };

            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT);

            // Combined: AltOffset1 = cos(15°) × cos(-15°) × AntennaHeight
            // cos(-15°) = cos(15°), so same as positive
            double expectedAltOffset = Math.Cos(15.0 * Math.PI / 180.0) * 
                                       Math.Cos(15.0 * Math.PI / 180.0) * 
                                       ANTENNA_HEIGHT; // ~3.9799m
            double expectedAltitude = (100.0 - expectedAltOffset);

            // Roll displacement: sin(15°) × 4.2672 = 1.1044m to the right (east)
            // Pitch displacement: sin(-15°) × 4.2672 = -1.1044m (backward/south)

            bool passed = Math.Abs(result.Altitude - expectedAltitude) < ALTITUDE_TOLERANCE &&
                          result.HasRTK == input.HasRTK &&
                          result.Heading == input.Heading &&
                          result.Speed == input.Speed;

            Console.WriteLine(passed ? "PASS" : "FAIL");
            return passed;
        }

        private bool TestRollAndPitch_PositivePitchNegativeRoll()
        {
            Console.Write("Test 19: Roll -15° Left + Pitch 15° (Nose Up)... ");

            SensorFusor fusor = new SensorFusor();
            GNSSFix input = CreateGNSSFix(
                -100.0,  // longitude degrees
                40.0,    // latitude degrees
                100.0,   // altitude meters
                0.0,     // heading (North)
                5.0,     // speed kph
                true
            );

            IMUValue imu = new IMUValue
            {
                Pitch = 15.0,   // Positive pitch (nose up, antenna moves forward)
                Roll = -15.0,   // Negative roll (rolling left, antenna moves left)
                Yaw = 0.0,
                YawRate = 0.0,
                Heading = 0.0
            };

            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT);

            // Combined: AltOffset1 = cos(-15°) × cos(15°) × AntennaHeight
            // cos(-15°) = cos(15°), so same as positive
            double expectedAltOffset = Math.Cos(15.0 * Math.PI / 180.0) * 
                                       Math.Cos(15.0 * Math.PI / 180.0) * 
                                       ANTENNA_HEIGHT; // ~3.9799m
            double expectedAltitude = (100.0 - expectedAltOffset);

            // Roll displacement: sin(-15°) × 4.2672 = -1.1044m (left/west)
            // Pitch displacement: sin(15°) × 4.2672 = 1.1044m forward (north)

            bool passed = Math.Abs(result.Altitude - expectedAltitude) < ALTITUDE_TOLERANCE &&
                          result.HasRTK == input.HasRTK &&
                          result.Heading == input.Heading &&
                          result.Speed == input.Speed;

            Console.WriteLine(passed ? "PASS" : "FAIL");
            return passed;
        }

        /// <summary>
        /// Compares two GNSSFix objects for equality (within tolerance)
        /// </summary>
        private bool AreEqual(GNSSFix a, GNSSFix b)
        {
            return Math.Abs(a.Latitude - b.Latitude) < POSITION_TOLERANCE &&
                   Math.Abs(a.Longitude - b.Longitude) < POSITION_TOLERANCE &&
                   Math.Abs(a.Altitude - b.Altitude) < ALTITUDE_TOLERANCE &&
                   Math.Abs(a.Heading - b.Heading) < HEADING_TOLERANCE &&
                   Math.Abs(a.Speed - b.Speed) < 0.1 &&
                   a.HasRTK == b.HasRTK;
        }
    }
}

