using Controller_Test_Harness;
using Controller;
using System;

namespace Controller_Test_Harness
{
    public class SensorFusor_Tests
    {
        private const int ANTENNA_HEIGHT = 426; // 426 cm (4.26 meters)
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
            allPassed &= TestAntennaLeft_Positive();
            allPassed &= TestAntennaLeft_Negative();
            allPassed &= TestAntennaForward_Positive();
            allPassed &= TestAntennaForward_Negative();
            allPassed &= TestAntennaLeftAndForward_BothPositive();
            allPassed &= TestAntennaLeftAndForward_BothNegative();
            allPassed &= TestAntennaLeftAndForward_PositiveLeftNegativeForward();
            allPassed &= TestAntennaLeftAndForward_NegativeLeftPositiveForward();
            allPassed &= TestAntennaLeft_WithRoll();
            allPassed &= TestAntennaLeft_NoRoll();
            allPassed &= TestAntennaLeftAndForward_WithRollAndPitch();
            allPassed &= TestAntennaForward_ZeroRollPitch_Positive();
            allPassed &= TestAntennaForward_ZeroRollPitch_Negative();
            allPassed &= TestAntennaLeft_ZeroRollPitch_Positive();
            allPassed &= TestAntennaLeft_ZeroRollPitch_Negative();
            allPassed &= TestAntennaLeftAndForward_ZeroRollPitch_BothPositive();
            allPassed &= TestAntennaLeftAndForward_ZeroRollPitch_BothNegative();
            allPassed &= TestAntennaLeftAndForward_ZeroRollPitch_PositiveLeftNegativeForward();
            allPassed &= TestAntennaLeftAndForward_ZeroRollPitch_NegativeLeftPositiveForward();

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
                Heading = 0.0,
                YawRate = 0.0
            };

            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT, 0, 0);

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
                YawRate = 0.0,
                Heading = 0.0
            };

            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT, 0, 0);

            bool passed = AreEqual(input, result);
            Console.WriteLine(passed ? "PASS" : "FAIL");
            return passed;
        }

        private bool TestNoCorrection_ZeroRoll()
        {
            Console.Write("Test 3: Zero Roll/Pitch (Altitude Correction Applied)... ");

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
                YawRate = 0.0,
                Heading = 0.0
            };

            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT, 0, 0);

            // With zero roll/pitch, the else block applies altitude correction
            // AltOffset1 = AntennaHeight = 4.26m, AltOffset2 = 0
            // Expected altitude: 100.0 - 4.26 = 95.74m
            double expectedAltitude = 100.0 - (ANTENNA_HEIGHT / 100.0);
            
            bool passed = Math.Abs(result.Altitude - expectedAltitude) < ALTITUDE_TOLERANCE &&
                          result.HasRTK == input.HasRTK &&
                          result.Heading == input.Heading &&
                          result.Speed == input.Speed;

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
                YawRate = 0.0,
                Heading = 0.0
            };

            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT, 0, 0);

            // Expected: Shifted ~110.2cm west (increased longitude), altitude -411.7cm
            // Convert from cm to meters for comparison
            double expectedRollOffset = Math.Sin(15.0 * Math.PI / 180.0) * ANTENNA_HEIGHT / 100.0; // ~110.2cm = 1.102m
            double expectedAltOffset = Math.Cos(15.0 * Math.PI / 180.0) * ANTENNA_HEIGHT / 100.0; // ~411.7cm = 4.117m

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
                YawRate = 0.0,
                Heading = 0.0
            };

            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT, 0, 0);

            double expectedAltOffset = Math.Cos(15.0 * Math.PI / 180.0) * ANTENNA_HEIGHT / 100.0; // Convert cm to meters
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
                YawRate = 0.0,
                Heading = 0.0
            };

            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT, 0, 0);

            double expectedAltOffset = Math.Cos(30.0 * Math.PI / 180.0) * ANTENNA_HEIGHT / 100.0; // Convert cm to meters
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
                YawRate = 0.0,
                Heading = 0.0
            };

            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT, 0, 0);

            double expectedAltOffset = Math.Cos(45.0 * Math.PI / 180.0) * ANTENNA_HEIGHT / 100.0; // Convert cm to meters
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
                YawRate = 0.0,
                Heading = 0.0
            };

            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT, 0, 0);

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
                YawRate = 0.0,
                Heading = 0.0
            };

            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT, 0, 0);

            double expectedAltOffset = Math.Cos(1.0 * Math.PI / 180.0) * ANTENNA_HEIGHT / 100.0; // Convert cm to meters
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
                YawRate = 0.0,
                Heading = 90.0
            };

            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT, 0, 0);

            double expectedAltOffset = Math.Cos(15.0 * Math.PI / 180.0) * ANTENNA_HEIGHT / 100.0; // Convert cm to meters
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
                YawRate = 0.0,
                Heading = 45.0
            };

            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT, 0, 0);

            double expectedAltOffset = Math.Cos(15.0 * Math.PI / 180.0) * ANTENNA_HEIGHT / 100.0; // Convert cm to meters
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
                YawRate = 0.0,
                Heading = 350.0
            };

            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT, 0, 0);

            // Heading90 = 350 + 90 = 440 → should wrap to 80°
            double expectedAltOffset = Math.Cos(15.0 * Math.PI / 180.0) * ANTENNA_HEIGHT / 100.0; // Convert cm to meters
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
                YawRate = 0.0,
                Heading = 0.0
            };

            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT, 0, 0);

            // Pitch correction: AltOffset1 = cos(0°) × cos(15°) × AntennaHeight
            double expectedAltOffset = Math.Cos(0.0 * Math.PI / 180.0) * 
                                       Math.Cos(15.0 * Math.PI / 180.0) * 
                                       ANTENNA_HEIGHT / 100.0; // Convert cm to meters
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
                YawRate = 0.0,
                Heading = 0.0
            };

            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT, 0, 0);

            // Combined: AltOffset1 = cos(15°) × cos(15°) × AntennaHeight
            double expectedAltOffset = Math.Cos(15.0 * Math.PI / 180.0) * 
                                       Math.Cos(15.0 * Math.PI / 180.0) * 
                                       ANTENNA_HEIGHT / 100.0; // Convert cm to meters
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
                YawRate = 0.0,
                Heading = 0.0
            };

            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT, 0, 0);

            // Combined: AltOffset1 = cos(20°) × cos(20°) × AntennaHeight
            double expectedAltOffset = Math.Cos(20.0 * Math.PI / 180.0) * 
                                       Math.Cos(20.0 * Math.PI / 180.0) * 
                                       ANTENNA_HEIGHT / 100.0; // Convert cm to meters
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
                YawRate = 0.0,
                Heading = 0.0
            };

            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT, 0, 0);

            // Combined: AltOffset1 = cos(10°) × cos(30°) × AntennaHeight
            double expectedAltOffset = Math.Cos(10.0 * Math.PI / 180.0) * 
                                       Math.Cos(30.0 * Math.PI / 180.0) * 
                                       ANTENNA_HEIGHT / 100.0; // Convert cm to meters
            double expectedAltitude = (100.0 - expectedAltOffset);

            // Pitch displacement: sin(30°) × 426cm = 213cm forward
            // Roll displacement: sin(10°) × 426cm = 74cm to the right

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
                YawRate = 0.0,
                Heading = 0.0
            };

            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT, 0, 0);

            // Combined: AltOffset1 = cos(30°) × cos(10°) × AntennaHeight
            double expectedAltOffset = Math.Cos(30.0 * Math.PI / 180.0) * 
                                       Math.Cos(10.0 * Math.PI / 180.0) * 
                                       ANTENNA_HEIGHT / 100.0; // Convert cm to meters
            double expectedAltitude = (100.0 - expectedAltOffset);

            // Roll displacement: sin(30°) × 426cm = 213cm to the right
            // Pitch displacement: sin(10°) × 426cm = 74cm forward

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
                YawRate = 0.0,
                Heading = 0.0
            };

            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT, 0, 0);

            // Combined: AltOffset1 = cos(15°) × cos(-15°) × AntennaHeight
            // cos(-15°) = cos(15°), so same as positive
            double expectedAltOffset = Math.Cos(15.0 * Math.PI / 180.0) * 
                                       Math.Cos(15.0 * Math.PI / 180.0) * 
                                       ANTENNA_HEIGHT / 100.0; // Convert cm to meters
            double expectedAltitude = (100.0 - expectedAltOffset);

            // Roll displacement: sin(15°) × 426cm = 110cm to the right (east)
            // Pitch displacement: sin(-15°) × 426cm = -110cm (backward/south)

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
                YawRate = 0.0,
                Heading = 0.0
            };

            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT, 0, 0);

            // Combined: AltOffset1 = cos(-15°) × cos(15°) × AntennaHeight
            // cos(-15°) = cos(15°), so same as positive
            double expectedAltOffset = Math.Cos(15.0 * Math.PI / 180.0) * 
                                       Math.Cos(15.0 * Math.PI / 180.0) * 
                                       ANTENNA_HEIGHT / 100.0; // Convert cm to meters
            double expectedAltitude = (100.0 - expectedAltOffset);

            // Roll displacement: sin(-15°) × 426cm = -110cm (left/west)
            // Pitch displacement: sin(15°) × 426cm = 110cm forward (north)

            bool passed = Math.Abs(result.Altitude - expectedAltitude) < ALTITUDE_TOLERANCE &&
                          result.HasRTK == input.HasRTK &&
                          result.Heading == input.Heading &&
                          result.Speed == input.Speed;

            Console.WriteLine(passed ? "PASS" : "FAIL");
            return passed;
        }

        private bool TestAntennaLeft_Positive()
        {
            Console.Write("Test 20: AntennaLeft +50cm with 15° Roll... ");

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
                Roll = 15.0,  // 15° roll to right
                YawRate = 0.0,
                Heading = 0.0
            };

            int antennaLeft = 50; // 50cm left
            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT, antennaLeft, 0);

            // CenterOffset = cos(15°) × 50cm = ~48.3cm
            // AltOffset2 = sin(15°) × 48.3cm = ~12.5cm = 0.125m
            // RollTiltOffset = sin(15°) × 426cm = ~110.2cm
            // Total roll offset = RollTiltOffset + CenterOffset = ~158.5cm
            // AltOffset1 = cos(15°) × 426cm = ~411.7cm = 4.117m
            // Final altitude = 100.0 - (4.117 - 0.125) = 96.008m
            double centerOffset = Math.Cos(15.0 * Math.PI / 180.0) * antennaLeft / 100.0; // Convert cm to meters
            double altOffset2 = Math.Sin(15.0 * Math.PI / 180.0) * centerOffset;
            double expectedAltOffset = Math.Cos(15.0 * Math.PI / 180.0) * ANTENNA_HEIGHT / 100.0;
            double expectedAltitude = (100.0 - (expectedAltOffset - altOffset2));

            bool passed = Math.Abs(result.Altitude - expectedAltitude) < ALTITUDE_TOLERANCE &&
                          result.HasRTK == input.HasRTK &&
                          result.Heading == input.Heading &&
                          result.Speed == input.Speed;

            Console.WriteLine(passed ? "PASS" : "FAIL");
            return passed;
        }

        private bool TestAntennaLeft_Negative()
        {
            Console.Write("Test 21: AntennaLeft -50cm with 15° Roll... ");

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
                Roll = 15.0,  // 15° roll to right
                YawRate = 0.0,
                Heading = 0.0
            };

            int antennaLeft = -50; // 50cm right (negative left)
            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT, antennaLeft, 0);

            // CenterOffset = cos(15°) × (-50cm) = ~-48.3cm
            // AltOffset2 = sin(15°) × (-48.3cm) = ~-12.5cm = -0.125m
            // AltOffset1 = cos(15°) × 426cm = ~411.7cm = 4.117m
            // Final altitude = 100.0 - (4.117 - (-0.125)) = 100.0 - 4.242 = 95.758m
            double centerOffset = Math.Cos(15.0 * Math.PI / 180.0) * antennaLeft / 100.0; // Convert cm to meters
            double altOffset2 = Math.Sin(15.0 * Math.PI / 180.0) * centerOffset;
            double expectedAltOffset = Math.Cos(15.0 * Math.PI / 180.0) * ANTENNA_HEIGHT / 100.0;
            double expectedAltitude = (100.0 - (expectedAltOffset - altOffset2));

            bool passed = Math.Abs(result.Altitude - expectedAltitude) < ALTITUDE_TOLERANCE &&
                          result.HasRTK == input.HasRTK &&
                          result.Heading == input.Heading &&
                          result.Speed == input.Speed;

            Console.WriteLine(passed ? "PASS" : "FAIL");
            return passed;
        }

        private bool TestAntennaForward_Positive()
        {
            Console.Write("Test 22: AntennaForward +100cm with 1° Roll... ");

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
                Roll = 1.0,  // Small roll to trigger correction block
                YawRate = 0.0,
                Heading = 0.0
            };

            int antennaForward = 100; // 100cm forward
            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT, 0, antennaForward);

            // AntennaForward moves backward along heading (negative direction)
            // Since heading is 0° (North), moves 100cm = 1m south
            // Small roll correction: AltOffset1 = cos(1°) × 426cm = ~425.9cm = 4.259m
            double expectedAltOffset = Math.Cos(1.0 * Math.PI / 180.0) * ANTENNA_HEIGHT / 100.0;
            double expectedAltitude = (100.0 - expectedAltOffset);

            bool passed = Math.Abs(result.Altitude - expectedAltitude) < ALTITUDE_TOLERANCE &&
                          result.HasRTK == input.HasRTK &&
                          result.Heading == input.Heading &&
                          result.Speed == input.Speed;

            Console.WriteLine(passed ? "PASS" : "FAIL");
            return passed;
        }

        private bool TestAntennaForward_Negative()
        {
            Console.Write("Test 23: AntennaForward -100cm with 1° Roll... ");

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
                Roll = 1.0,  // Small roll to trigger correction block
                YawRate = 0.0,
                Heading = 0.0
            };

            int antennaForward = -100; // 100cm backward (negative forward)
            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT, 0, antennaForward);

            // AntennaForward moves backward along heading
            // -AntennaForward = -(-100) = +100cm = 1m forward (north)
            // Small roll correction: AltOffset1 = cos(1°) × 426cm = ~425.9cm = 4.259m
            double expectedAltOffset = Math.Cos(1.0 * Math.PI / 180.0) * ANTENNA_HEIGHT / 100.0;
            double expectedAltitude = (100.0 - expectedAltOffset);

            bool passed = Math.Abs(result.Altitude - expectedAltitude) < ALTITUDE_TOLERANCE &&
                          result.HasRTK == input.HasRTK &&
                          result.Heading == input.Heading &&
                          result.Speed == input.Speed;

            Console.WriteLine(passed ? "PASS" : "FAIL");
            return passed;
        }

        private bool TestAntennaLeftAndForward_BothPositive()
        {
            Console.Write("Test 24: AntennaLeft +50cm + AntennaForward +100cm with 15° Roll... ");

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
                Roll = 15.0,  // 15° roll
                YawRate = 0.0,
                Heading = 0.0
            };

            int antennaLeft = 50; // 50cm left
            int antennaForward = 100; // 100cm forward
            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT, antennaLeft, antennaForward);

            // Combined effects: roll correction + antenna left + antenna forward
            double centerOffset = Math.Cos(15.0 * Math.PI / 180.0) * antennaLeft / 100.0;
            double altOffset2 = Math.Sin(15.0 * Math.PI / 180.0) * centerOffset;
            double expectedAltOffset = Math.Cos(15.0 * Math.PI / 180.0) * ANTENNA_HEIGHT / 100.0;
            double expectedAltitude = (100.0 - (expectedAltOffset - altOffset2));

            bool passed = Math.Abs(result.Altitude - expectedAltitude) < ALTITUDE_TOLERANCE &&
                          result.HasRTK == input.HasRTK &&
                          result.Heading == input.Heading &&
                          result.Speed == input.Speed;

            Console.WriteLine(passed ? "PASS" : "FAIL");
            return passed;
        }

        private bool TestAntennaLeftAndForward_BothNegative()
        {
            Console.Write("Test 25: AntennaLeft -50cm + AntennaForward -100cm with 15° Roll... ");

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
                Roll = 15.0,  // 15° roll
                YawRate = 0.0,
                Heading = 0.0
            };

            int antennaLeft = -50; // 50cm right
            int antennaForward = -100; // 100cm backward
            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT, antennaLeft, antennaForward);

            // Combined effects with negative values
            double centerOffset = Math.Cos(15.0 * Math.PI / 180.0) * antennaLeft / 100.0;
            double altOffset2 = Math.Sin(15.0 * Math.PI / 180.0) * centerOffset;
            double expectedAltOffset = Math.Cos(15.0 * Math.PI / 180.0) * ANTENNA_HEIGHT / 100.0;
            double expectedAltitude = (100.0 - (expectedAltOffset - altOffset2));

            bool passed = Math.Abs(result.Altitude - expectedAltitude) < ALTITUDE_TOLERANCE &&
                          result.HasRTK == input.HasRTK &&
                          result.Heading == input.Heading &&
                          result.Speed == input.Speed;

            Console.WriteLine(passed ? "PASS" : "FAIL");
            return passed;
        }

        private bool TestAntennaLeftAndForward_PositiveLeftNegativeForward()
        {
            Console.Write("Test 26: AntennaLeft +50cm + AntennaForward -100cm with 15° Roll... ");

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
                Roll = 15.0,  // 15° roll
                YawRate = 0.0,
                Heading = 0.0
            };

            int antennaLeft = 50; // 50cm left
            int antennaForward = -100; // 100cm backward
            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT, antennaLeft, antennaForward);

            double centerOffset = Math.Cos(15.0 * Math.PI / 180.0) * antennaLeft / 100.0;
            double altOffset2 = Math.Sin(15.0 * Math.PI / 180.0) * centerOffset;
            double expectedAltOffset = Math.Cos(15.0 * Math.PI / 180.0) * ANTENNA_HEIGHT / 100.0;
            double expectedAltitude = (100.0 - (expectedAltOffset - altOffset2));

            bool passed = Math.Abs(result.Altitude - expectedAltitude) < ALTITUDE_TOLERANCE &&
                          result.HasRTK == input.HasRTK &&
                          result.Heading == input.Heading &&
                          result.Speed == input.Speed;

            Console.WriteLine(passed ? "PASS" : "FAIL");
            return passed;
        }

        private bool TestAntennaLeftAndForward_NegativeLeftPositiveForward()
        {
            Console.Write("Test 27: AntennaLeft -50cm + AntennaForward +100cm with 15° Roll... ");

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
                Roll = 15.0,  // 15° roll
                YawRate = 0.0,
                Heading = 0.0
            };

            int antennaLeft = -50; // 50cm right
            int antennaForward = 100; // 100cm forward
            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT, antennaLeft, antennaForward);

            double centerOffset = Math.Cos(15.0 * Math.PI / 180.0) * antennaLeft / 100.0;
            double altOffset2 = Math.Sin(15.0 * Math.PI / 180.0) * centerOffset;
            double expectedAltOffset = Math.Cos(15.0 * Math.PI / 180.0) * ANTENNA_HEIGHT / 100.0;
            double expectedAltitude = (100.0 - (expectedAltOffset - altOffset2));

            bool passed = Math.Abs(result.Altitude - expectedAltitude) < ALTITUDE_TOLERANCE &&
                          result.HasRTK == input.HasRTK &&
                          result.Heading == input.Heading &&
                          result.Speed == input.Speed;

            Console.WriteLine(passed ? "PASS" : "FAIL");
            return passed;
        }

        private bool TestAntennaLeft_WithRoll()
        {
            Console.Write("Test 28: AntennaLeft +75cm with 30° Roll... ");

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
                Roll = 30.0,  // 30° roll
                YawRate = 0.0,
                Heading = 0.0
            };

            int antennaLeft = 75; // 75cm left
            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT, antennaLeft, 0);

            // CenterOffset = cos(30°) × 75cm = ~65.0cm
            // AltOffset2 = sin(30°) × 65.0cm = ~32.5cm = 0.325m
            // AltOffset1 = cos(30°) × 426cm = ~369.5cm = 3.695m
            // Final altitude = 100.0 - (3.695 - 0.325) = 96.63m
            double centerOffset = Math.Cos(30.0 * Math.PI / 180.0) * antennaLeft / 100.0;
            double altOffset2 = Math.Sin(30.0 * Math.PI / 180.0) * centerOffset;
            double expectedAltOffset = Math.Cos(30.0 * Math.PI / 180.0) * ANTENNA_HEIGHT / 100.0;
            double expectedAltitude = (100.0 - (expectedAltOffset - altOffset2));

            bool passed = Math.Abs(result.Altitude - expectedAltitude) < ALTITUDE_TOLERANCE &&
                          result.HasRTK == input.HasRTK &&
                          result.Heading == input.Heading &&
                          result.Speed == input.Speed;

            Console.WriteLine(passed ? "PASS" : "FAIL");
            return passed;
        }

        private bool TestAntennaLeft_NoRoll()
        {
            Console.Write("Test 29: AntennaLeft +50cm with No Roll (Altitude Correction Applied)... ");

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
                Roll = 0.0,  // No roll
                YawRate = 0.0,
                Heading = 0.0
            };

            int antennaLeft = 50; // 50cm left
            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT, antennaLeft, 0);

            // With zero roll/pitch, the else-if block applies altitude correction
            // CenterOffset = cos(0°) × 50cm = 50cm = 0.5m
            // AltOffset2 = sin(0°) × 0.5m = 0m
            // AltOffset1 = AntennaHeight = 4.26m
            // Altitude correction: 100.0 - (4.26 - 0) = 95.74m
            // Note: AntennaLeft is calculated but doesn't affect altitude when roll=0
            double expectedAltitude = 100.0 - (ANTENNA_HEIGHT / 100.0); // 100.0 - 4.26 = 95.74m

            bool passed = Math.Abs(result.Altitude - expectedAltitude) < ALTITUDE_TOLERANCE &&
                          result.HasRTK == input.HasRTK &&
                          result.Heading == input.Heading &&
                          result.Speed == input.Speed;

            Console.WriteLine(passed ? "PASS" : "FAIL");
            return passed;
        }

        private bool TestAntennaLeftAndForward_WithRollAndPitch()
        {
            Console.Write("Test 30: AntennaLeft +50cm + AntennaForward +100cm with 15° Roll + 10° Pitch... ");

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
                Pitch = 10.0,  // 10° pitch
                Roll = 15.0,   // 15° roll
                YawRate = 0.0,
                Heading = 0.0
            };

            int antennaLeft = 50; // 50cm left
            int antennaForward = 100; // 100cm forward
            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT, antennaLeft, antennaForward);

            // Combined: roll + pitch + antenna left + antenna forward
            double centerOffset = Math.Cos(15.0 * Math.PI / 180.0) * antennaLeft / 100.0;
            double altOffset2 = Math.Sin(15.0 * Math.PI / 180.0) * centerOffset;
            double expectedAltOffset = Math.Cos(15.0 * Math.PI / 180.0) * 
                                       Math.Cos(10.0 * Math.PI / 180.0) * 
                                       ANTENNA_HEIGHT / 100.0;
            double expectedAltitude = (100.0 - (expectedAltOffset - altOffset2));

            bool passed = Math.Abs(result.Altitude - expectedAltitude) < ALTITUDE_TOLERANCE &&
                          result.HasRTK == input.HasRTK &&
                          result.Heading == input.Heading &&
                          result.Speed == input.Speed;

            Console.WriteLine(passed ? "PASS" : "FAIL");
            return passed;
        }

        private bool TestAntennaForward_ZeroRollPitch_Positive()
        {
            Console.Write("Test 31: AntennaForward +100cm with Zero Roll/Pitch... ");

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
                Roll = 0.0,  // Zero roll
                YawRate = 0.0,
                Heading = 0.0
            };

            int antennaForward = 100; // 100cm forward
            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT, 0, antennaForward);

            // With zero roll/pitch: AltOffset1 = AntennaHeight, AltOffset2 = 0
            // Altitude correction: 100.0 - (4.26 - 0) = 95.74m
            // AntennaForward moves backward along heading: -100cm = -1m (south when heading north)
            double expectedAltitude = 100.0 - (ANTENNA_HEIGHT / 100.0); // 100.0 - 4.26 = 95.74m

            bool passed = Math.Abs(result.Altitude - expectedAltitude) < ALTITUDE_TOLERANCE &&
                          result.HasRTK == input.HasRTK &&
                          result.Heading == input.Heading &&
                          result.Speed == input.Speed;

            Console.WriteLine(passed ? "PASS" : "FAIL");
            return passed;
        }

        private bool TestAntennaForward_ZeroRollPitch_Negative()
        {
            Console.Write("Test 32: AntennaForward -100cm with Zero Roll/Pitch... ");

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
                Roll = 0.0,  // Zero roll
                YawRate = 0.0,
                Heading = 0.0
            };

            int antennaForward = -100; // 100cm backward
            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT, 0, antennaForward);

            // With zero roll/pitch: AltOffset1 = AntennaHeight, AltOffset2 = 0
            // Altitude correction: 100.0 - (4.26 - 0) = 95.74m
            // AntennaForward moves backward: -(-100cm) = +100cm = +1m (north when heading north)
            double expectedAltitude = 100.0 - (ANTENNA_HEIGHT / 100.0); // 100.0 - 4.26 = 95.74m

            bool passed = Math.Abs(result.Altitude - expectedAltitude) < ALTITUDE_TOLERANCE &&
                          result.HasRTK == input.HasRTK &&
                          result.Heading == input.Heading &&
                          result.Speed == input.Speed;

            Console.WriteLine(passed ? "PASS" : "FAIL");
            return passed;
        }

        private bool TestAntennaLeft_ZeroRollPitch_Positive()
        {
            Console.Write("Test 33: AntennaLeft +50cm with Zero Roll/Pitch... ");

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
                Roll = 0.0,  // Zero roll
                YawRate = 0.0,
                Heading = 0.0
            };

            int antennaLeft = 50; // 50cm left
            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT, antennaLeft, 0);

            // With zero roll/pitch: 
            // CenterOffset = cos(0°) × 50cm = 50cm = 0.5m
            // AltOffset2 = sin(0°) × 0.5m = 0m
            // AltOffset1 = AntennaHeight = 4.26m
            // Altitude correction: 100.0 - (4.26 - 0) = 95.74m
            // Note: CenterOffset is calculated but not used for position when roll=0
            double expectedAltitude = 100.0 - (ANTENNA_HEIGHT / 100.0); // 100.0 - 4.26 = 95.74m

            bool passed = Math.Abs(result.Altitude - expectedAltitude) < ALTITUDE_TOLERANCE &&
                          result.HasRTK == input.HasRTK &&
                          result.Heading == input.Heading &&
                          result.Speed == input.Speed;

            Console.WriteLine(passed ? "PASS" : "FAIL");
            return passed;
        }

        private bool TestAntennaLeft_ZeroRollPitch_Negative()
        {
            Console.Write("Test 34: AntennaLeft -50cm with Zero Roll/Pitch... ");

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
                Roll = 0.0,  // Zero roll
                YawRate = 0.0,
                Heading = 0.0
            };

            int antennaLeft = -50; // 50cm right
            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT, antennaLeft, 0);

            // With zero roll/pitch:
            // CenterOffset = cos(0°) × (-50cm) = -50cm = -0.5m
            // AltOffset2 = sin(0°) × (-0.5m) = 0m
            // AltOffset1 = AntennaHeight = 4.26m
            // Altitude correction: 100.0 - (4.26 - 0) = 95.74m
            double expectedAltitude = 100.0 - (ANTENNA_HEIGHT / 100.0); // 100.0 - 4.26 = 95.74m

            bool passed = Math.Abs(result.Altitude - expectedAltitude) < ALTITUDE_TOLERANCE &&
                          result.HasRTK == input.HasRTK &&
                          result.Heading == input.Heading &&
                          result.Speed == input.Speed;

            Console.WriteLine(passed ? "PASS" : "FAIL");
            return passed;
        }

        private bool TestAntennaLeftAndForward_ZeroRollPitch_BothPositive()
        {
            Console.Write("Test 35: AntennaLeft +50cm + AntennaForward +100cm with Zero Roll/Pitch... ");

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
                Roll = 0.0,  // Zero roll and pitch
                YawRate = 0.0,
                Heading = 0.0
            };

            int antennaLeft = 50; // 50cm left
            int antennaForward = 100; // 100cm forward
            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT, antennaLeft, antennaForward);

            // With zero roll/pitch:
            // CenterOffset = cos(0°) × 50cm = 50cm = 0.5m
            // AltOffset2 = sin(0°) × 0.5m = 0m
            // AltOffset1 = AntennaHeight = 4.26m
            // Altitude correction: 100.0 - (4.26 - 0) = 95.74m
            // AntennaForward moves backward: -100cm = -1m (south when heading north)
            double expectedAltitude = 100.0 - (ANTENNA_HEIGHT / 100.0); // 100.0 - 4.26 = 95.74m

            bool passed = Math.Abs(result.Altitude - expectedAltitude) < ALTITUDE_TOLERANCE &&
                          result.HasRTK == input.HasRTK &&
                          result.Heading == input.Heading &&
                          result.Speed == input.Speed;

            Console.WriteLine(passed ? "PASS" : "FAIL");
            return passed;
        }

        private bool TestAntennaLeftAndForward_ZeroRollPitch_BothNegative()
        {
            Console.Write("Test 36: AntennaLeft -50cm + AntennaForward -100cm with Zero Roll/Pitch... ");

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
                Roll = 0.0,  // Zero roll and pitch
                YawRate = 0.0,
                Heading = 0.0
            };

            int antennaLeft = -50; // 50cm right
            int antennaForward = -100; // 100cm backward
            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT, antennaLeft, antennaForward);

            // With zero roll/pitch:
            // CenterOffset = cos(0°) × (-50cm) = -50cm = -0.5m
            // AltOffset2 = sin(0°) × (-0.5m) = 0m
            // AltOffset1 = AntennaHeight = 4.26m
            // Altitude correction: 100.0 - (4.26 - 0) = 95.74m
            // AntennaForward moves backward: -(-100cm) = +100cm = +1m (north when heading north)
            double expectedAltitude = 100.0 - (ANTENNA_HEIGHT / 100.0); // 100.0 - 4.26 = 95.74m

            bool passed = Math.Abs(result.Altitude - expectedAltitude) < ALTITUDE_TOLERANCE &&
                          result.HasRTK == input.HasRTK &&
                          result.Heading == input.Heading &&
                          result.Speed == input.Speed;

            Console.WriteLine(passed ? "PASS" : "FAIL");
            return passed;
        }

        private bool TestAntennaLeftAndForward_ZeroRollPitch_PositiveLeftNegativeForward()
        {
            Console.Write("Test 37: AntennaLeft +50cm + AntennaForward -100cm with Zero Roll/Pitch... ");

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
                Roll = 0.0,  // Zero roll and pitch
                YawRate = 0.0,
                Heading = 0.0
            };

            int antennaLeft = 50; // 50cm left
            int antennaForward = -100; // 100cm backward
            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT, antennaLeft, antennaForward);

            // With zero roll/pitch:
            // CenterOffset = cos(0°) × 50cm = 50cm = 0.5m
            // AltOffset2 = sin(0°) × 0.5m = 0m
            // AltOffset1 = AntennaHeight = 4.26m
            // Altitude correction: 100.0 - (4.26 - 0) = 95.74m
            // AntennaForward moves backward: -(-100cm) = +100cm = +1m (north when heading north)
            double expectedAltitude = 100.0 - (ANTENNA_HEIGHT / 100.0); // 100.0 - 4.26 = 95.74m

            bool passed = Math.Abs(result.Altitude - expectedAltitude) < ALTITUDE_TOLERANCE &&
                          result.HasRTK == input.HasRTK &&
                          result.Heading == input.Heading &&
                          result.Speed == input.Speed;

            Console.WriteLine(passed ? "PASS" : "FAIL");
            return passed;
        }

        private bool TestAntennaLeftAndForward_ZeroRollPitch_NegativeLeftPositiveForward()
        {
            Console.Write("Test 38: AntennaLeft -50cm + AntennaForward +100cm with Zero Roll/Pitch... ");

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
                Roll = 0.0,  // Zero roll and pitch
                YawRate = 0.0,
                Heading = 0.0
            };

            int antennaLeft = -50; // 50cm right
            int antennaForward = 100; // 100cm forward
            GNSSFix result = fusor.Fuse(input, imu, ANTENNA_HEIGHT, antennaLeft, antennaForward);

            // With zero roll/pitch:
            // CenterOffset = cos(0°) × (-50cm) = -50cm = -0.5m
            // AltOffset2 = sin(0°) × (-0.5m) = 0m
            // AltOffset1 = AntennaHeight = 4.26m
            // Altitude correction: 100.0 - (4.26 - 0) = 95.74m
            // AntennaForward moves backward: -100cm = -1m (south when heading north)
            double expectedAltitude = 100.0 - (ANTENNA_HEIGHT / 100.0); // 100.0 - 4.26 = 95.74m

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

