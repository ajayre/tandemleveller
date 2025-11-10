using Controller_Test_Harness;
using System;

namespace Controller_Test_Harness
{
    public class NMEAParsing_Tests
    {
        private const double POSITION_TOLERANCE = 0.0000001; // degrees (for lat/lon differences)
        private const double ALTITUDE_TOLERANCE = 0.01; // meters

        /// <summary>
        /// Runs all tests and returns true if all pass, false if any fail
        /// </summary>
        public bool Run()
        {
            bool allPassed = true;

            Console.WriteLine("Running NMEA Parsing Tests...");
            Console.WriteLine("============================\n");

            // Test valid GNGGA sentences
            allPassed &= TestValidGNGGA_014638();
            allPassed &= TestValidGNGGA_014639();
            allPassed &= TestValidGNGGA_014640();
            allPassed &= TestValidGNGGA_014641();
            allPassed &= TestValidGNGGA_014642();
            allPassed &= TestValidGNGGA_014643();
            allPassed &= TestValidGNGGA_014644();
            allPassed &= TestValidGNGGA_014645();
            allPassed &= TestValidGNGGA_014646();
            allPassed &= TestValidGNGGA_014647();
            allPassed &= TestValidGNGGA_014648();
            allPassed &= TestValidGNGGA_014649();
            allPassed &= TestValidGNGGA_014650();
            allPassed &= TestValidGNGGA_014651();
            allPassed &= TestValidGNGGA_014652();
            allPassed &= TestValidGNGGA_014653();
            allPassed &= TestValidGNGGA_014654();
            allPassed &= TestValidGNGGA_014655();
            allPassed &= TestValidGNGGA_014656();
            allPassed &= TestValidGNGGA_014657();
            allPassed &= TestValidGNGGA_014658();
            allPassed &= TestValidGNGGA_014659();
            allPassed &= TestValidGNGGA_014700();
            allPassed &= TestValidGNGGA_014701();
            allPassed &= TestValidGNGGA_014702();
            allPassed &= TestValidGNGGA_014703();
            allPassed &= TestValidGNGGA_014704();
            allPassed &= TestValidGNGGA_014705();
            allPassed &= TestValidGNGGA_014706();
            allPassed &= TestValidGNGGA_014707();
            allPassed &= TestValidGNGGA_014708();
            allPassed &= TestValidGNGGA_014709();
            allPassed &= TestValidGNGGA_014710();
            allPassed &= TestValidGNGGA_014711();
            allPassed &= TestValidGNGGA_014712();
            allPassed &= TestValidGNGGA_014713();
            allPassed &= TestValidGNGGA_014714();
            allPassed &= TestValidGNGGA_014715();
            allPassed &= TestValidGNGGA_014716();
            allPassed &= TestValidGNGGA_014717();
            allPassed &= TestValidGNGGA_014718();

            // Test RTK Fix (quality 4)
            allPassed &= TestRTKFix_Quality4();
            allPassed &= TestRTKFix_Quality4_WithXFill();
            
            // Test RTK Float (quality 5)
            allPassed &= TestRTKFloat_Quality5();
            allPassed &= TestRTKFloat_Quality5_OmniSTAR();
            
            // Test No RTK cases (quality 0, 1, 2, 3, 6)
            allPassed &= TestNoRTK_Quality0_Invalid();
            allPassed &= TestNoRTK_Quality1_GPS();
            allPassed &= TestNoRTK_Quality2_DGPS();
            allPassed &= TestNoRTK_Quality3_NotApplicable();
            allPassed &= TestNoRTK_Quality6_INS();
            
            // Test error cases
            allPassed &= TestNullString();
            allPassed &= TestEmptyString();
            allPassed &= TestMissingDollarSign();
            allPassed &= TestUnsupportedSentenceType();
            allPassed &= TestIncompleteSentence();
            allPassed &= TestMissingLatitude();
            allPassed &= TestMissingLongitude();
            allPassed &= TestInvalidLatitudeFormat();
            allPassed &= TestInvalidLongitudeFormat();
            allPassed &= TestInvalidDirectionIndicator();
            
            // Test additional mangled/invalid strings
            allPassed &= TestOnlyDollarSign();
            allPassed &= TestOnlySentenceType();
            allPassed &= TestChecksumOnly();
            allPassed &= TestLatitudeOutOfRange();
            allPassed &= TestLongitudeOutOfRange();
            allPassed &= TestLatitudeMinutesOutOfRange();
            allPassed &= TestLongitudeMinutesOutOfRange();
            allPassed &= TestInvalidAltitudeFormat();
            allPassed &= TestInvalidQualityFormat();
            allPassed &= TestMissingLatitudeDirection();
            allPassed &= TestMissingLongitudeDirection();
            allPassed &= TestWhitespaceOnly();
            allPassed &= TestExtraCommas();
            allPassed &= TestInvalidCharactersInLatitude();
            allPassed &= TestInvalidCharactersInLongitude();

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

        private bool TestValidGNGGA_014638()
        {
            Console.Write("Test: Valid GNGGA 014638... ");
            try
            {
                string nmea = "$GNGGA,014638.00,3617.3591173,N,09430.9142232,W,2,12,0.50,372.743,M,-27.832,M,,0131*72";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Latitude - 36.2893186) < POSITION_TOLERANCE &&
                              Math.Abs(fix.Longitude - (-94.5152371)) < POSITION_TOLERANCE &&
                              Math.Abs(fix.Altitude - 372.743) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None &&
                              !fix.HasRTK;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestValidGNGGA_014639()
        {
            Console.Write("Test: Valid GNGGA 014639... ");
            try
            {
                string nmea = "$GNGGA,014639.00,3617.3591263,N,09430.9142375,W,2,12,0.49,372.711,M,-27.832,M,,0131*7C";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Latitude - 36.2893188) < POSITION_TOLERANCE &&
                              Math.Abs(fix.Longitude - (-94.5152373)) < POSITION_TOLERANCE &&
                              Math.Abs(fix.Altitude - 372.711) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None &&
                              !fix.HasRTK;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestValidGNGGA_014640()
        {
            Console.Write("Test: Valid GNGGA 014640... ");
            try
            {
                string nmea = "$GNGGA,014640.00,3617.3591255,N,09430.9142455,W,2,12,0.49,372.666,M,-27.832,M,,0131*73";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Latitude - 36.2893188) < POSITION_TOLERANCE &&
                              Math.Abs(fix.Longitude - (-94.5152374)) < POSITION_TOLERANCE &&
                              Math.Abs(fix.Altitude - 372.666) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None &&
                              !fix.HasRTK;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestValidGNGGA_014641()
        {
            Console.Write("Test: Valid GNGGA 014641... ");
            try
            {
                string nmea = "$GNGGA,014641.00,3617.3591317,N,09430.9142498,W,2,12,0.49,372.648,M,-27.832,M,,0131*78";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Altitude - 372.648) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None &&
                              !fix.HasRTK;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestValidGNGGA_014642()
        {
            Console.Write("Test: Valid GNGGA 014642... ");
            try
            {
                string nmea = "$GNGGA,014642.00,3617.3591403,N,09430.9142630,W,2,12,0.49,372.627,M,-27.832,M,,0131*70";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Altitude - 372.627) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestValidGNGGA_014643()
        {
            Console.Write("Test: Valid GNGGA 014643... ");
            try
            {
                string nmea = "$GNGGA,014643.00,3617.3591652,N,09430.9142897,W,2,12,0.49,372.622,M,-27.832,M,,0131*71";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Altitude - 372.622) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestValidGNGGA_014644()
        {
            Console.Write("Test: Valid GNGGA 014644... ");
            try
            {
                string nmea = "$GNGGA,014644.00,3617.3591774,N,09430.9143017,W,2,12,0.50,372.587,M,-27.832,M,,0131*76";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Altitude - 372.587) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestValidGNGGA_014645()
        {
            Console.Write("Test: Valid GNGGA 014645... ");
            try
            {
                string nmea = "$GNGGA,014645.00,3617.3591946,N,09430.9143128,W,2,12,0.49,372.594,M,-27.832,M,,0131*7F";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Altitude - 372.594) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestValidGNGGA_014646()
        {
            Console.Write("Test: Valid GNGGA 014646... ");
            try
            {
                string nmea = "$GNGGA,014646.00,3617.3592074,N,09430.9143233,W,2,12,0.49,372.600,M,-27.832,M,,0131*70";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Altitude - 372.600) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestValidGNGGA_014647()
        {
            Console.Write("Test: Valid GNGGA 014647... ");
            try
            {
                string nmea = "$GNGGA,014647.00,3617.3592229,N,09430.9143364,W,2,12,0.49,372.618,M,-27.832,M,,0131*71";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Altitude - 372.618) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestValidGNGGA_014648()
        {
            Console.Write("Test: Valid GNGGA 014648... ");
            try
            {
                string nmea = "$GNGGA,014648.00,3617.3592269,N,09430.9143391,W,2,12,0.49,372.566,M,-27.832,M,,0131*7A";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Altitude - 372.566) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestValidGNGGA_014649()
        {
            Console.Write("Test: Valid GNGGA 014649... ");
            try
            {
                string nmea = "$GNGGA,014649.00,3617.3592411,N,09430.9143546,W,2,12,0.49,372.578,M,-27.832,M,,0131*71";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Altitude - 372.578) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestValidGNGGA_014650()
        {
            Console.Write("Test: Valid GNGGA 014650... ");
            try
            {
                string nmea = "$GNGGA,014650.00,3617.3592566,N,09430.9143694,W,2,12,0.49,372.593,M,-27.832,M,,0131*71";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Altitude - 372.593) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestValidGNGGA_014651()
        {
            Console.Write("Test: Valid GNGGA 014651... ");
            try
            {
                string nmea = "$GNGGA,014651.00,3617.3592708,N,09430.9143813,W,2,12,0.49,372.627,M,-27.832,M,,0131*77";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Altitude - 372.627) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestValidGNGGA_014652()
        {
            Console.Write("Test: Valid GNGGA 014652... ");
            try
            {
                string nmea = "$GNGGA,014652.00,3617.3592871,N,09430.9143965,W,2,12,0.49,372.669,M,-27.832,M,,0131*7F";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Altitude - 372.669) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestValidGNGGA_014653()
        {
            Console.Write("Test: Valid GNGGA 014653... ");
            try
            {
                string nmea = "$GNGGA,014653.00,3617.3593049,N,09430.9144099,W,2,12,0.49,372.687,M,-27.832,M,,0131*71";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Altitude - 372.687) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestValidGNGGA_014654()
        {
            Console.Write("Test: Valid GNGGA 014654... ");
            try
            {
                string nmea = "$GNGGA,014654.00,3617.3593190,N,09430.9144199,W,2,12,0.49,372.703,M,-27.832,M,,0131*7F";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Altitude - 372.703) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestValidGNGGA_014655()
        {
            Console.Write("Test: Valid GNGGA 014655... ");
            try
            {
                string nmea = "$GNGGA,014655.00,3617.3593360,N,09430.9144345,W,2,12,0.49,372.683,M,-27.832,M,,0131*79";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Altitude - 372.683) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestValidGNGGA_014656()
        {
            Console.Write("Test: Valid GNGGA 014656... ");
            try
            {
                string nmea = "$GNGGA,014656.00,3617.3593503,N,09430.9144573,W,2,12,0.49,372.684,M,-27.832,M,,0131*7D";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Altitude - 372.684) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestValidGNGGA_014657()
        {
            Console.Write("Test: Valid GNGGA 014657... ");
            try
            {
                string nmea = "$GNGGA,014657.00,3617.3593677,N,09430.9144784,W,2,12,0.49,372.673,M,-27.832,M,,0131*7E";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Altitude - 372.673) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestValidGNGGA_014658()
        {
            Console.Write("Test: Valid GNGGA 014658... ");
            try
            {
                string nmea = "$GNGGA,014658.00,3617.3593891,N,09430.9145005,W,2,12,0.49,372.675,M,-27.832,M,,0131*7E";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Altitude - 372.675) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestValidGNGGA_014659()
        {
            Console.Write("Test: Valid GNGGA 014659... ");
            try
            {
                string nmea = "$GNGGA,014659.00,3617.3594047,N,09430.9145185,W,2,12,0.49,372.690,M,-27.832,M,,0131*79";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Altitude - 372.690) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestValidGNGGA_014700()
        {
            Console.Write("Test: Valid GNGGA 014700... ");
            try
            {
                string nmea = "$GNGGA,014700.00,3617.3594239,N,09430.9145342,W,2,12,0.49,372.711,M,-27.832,M,,0131*7E";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Altitude - 372.711) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestValidGNGGA_014701()
        {
            Console.Write("Test: Valid GNGGA 014701... ");
            try
            {
                string nmea = "$GNGGA,014701.00,3617.3594414,N,09430.9145469,W,2,12,0.49,372.717,M,-27.832,M,,0131*7E";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Altitude - 372.717) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestValidGNGGA_014702()
        {
            Console.Write("Test: Valid GNGGA 014702... ");
            try
            {
                string nmea = "$GNGGA,014702.00,3617.3594573,N,09430.9145621,W,2,12,0.49,372.712,M,-27.832,M,,0131*76";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Altitude - 372.712) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestValidGNGGA_014703()
        {
            Console.Write("Test: Valid GNGGA 014703... ");
            try
            {
                string nmea = "$GNGGA,014703.00,3617.3594738,N,09430.9145752,W,2,12,0.49,372.715,M,-27.832,M,,0131*78";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Altitude - 372.715) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestValidGNGGA_014704()
        {
            Console.Write("Test: Valid GNGGA 014704... ");
            try
            {
                string nmea = "$GNGGA,014704.00,3617.3595014,N,09430.9145913,W,2,12,0.49,372.742,M,-27.832,M,,0131*7E";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Altitude - 372.742) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestValidGNGGA_014705()
        {
            Console.Write("Test: Valid GNGGA 014705... ");
            try
            {
                string nmea = "$GNGGA,014705.00,3617.3595246,N,09430.9146037,W,2,12,0.49,372.761,M,-27.832,M,,0131*77";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Altitude - 372.761) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestValidGNGGA_014706()
        {
            Console.Write("Test: Valid GNGGA 014706... ");
            try
            {
                string nmea = "$GNGGA,014706.00,3617.3595457,N,09430.9146142,W,2,12,0.49,372.771,M,-27.832,M,,0131*70";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Altitude - 372.771) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestValidGNGGA_014707()
        {
            Console.Write("Test: Valid GNGGA 014707... ");
            try
            {
                string nmea = "$GNGGA,014707.00,3617.3595727,N,09430.9146259,W,2,12,0.49,372.786,M,-27.832,M,,0131*74";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Altitude - 372.786) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestValidGNGGA_014708()
        {
            Console.Write("Test: Valid GNGGA 014708... ");
            try
            {
                string nmea = "$GNGGA,014708.00,3617.3595966,N,09430.9146336,W,2,12,0.49,372.808,M,-27.832,M,,0131*71";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Altitude - 372.808) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestValidGNGGA_014709()
        {
            Console.Write("Test: Valid GNGGA 014709... ");
            try
            {
                string nmea = "$GNGGA,014709.00,3617.3596195,N,09430.9146368,W,2,12,0.49,372.844,M,-27.832,M,,0131*74";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Altitude - 372.844) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestValidGNGGA_014710()
        {
            Console.Write("Test: Valid GNGGA 014710... ");
            try
            {
                string nmea = "$GNGGA,014710.00,3617.3596273,N,09430.9146326,W,2,12,0.49,372.877,M,-27.832,M,,0131*7D";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Altitude - 372.877) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestValidGNGGA_014711()
        {
            Console.Write("Test: Valid GNGGA 014711... ");
            try
            {
                string nmea = "$GNGGA,014711.00,3617.3596489,N,09430.9146415,W,2,12,0.49,372.900,M,-27.832,M,,0131*79";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Altitude - 372.900) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestValidGNGGA_014712()
        {
            Console.Write("Test: Valid GNGGA 014712... ");
            try
            {
                string nmea = "$GNGGA,014712.00,3617.3596749,N,09430.9146584,W,2,12,0.49,372.942,M,-27.832,M,,0131*7A";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Altitude - 372.942) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestValidGNGGA_014713()
        {
            Console.Write("Test: Valid GNGGA 014713... ");
            try
            {
                string nmea = "$GNGGA,014713.00,3617.3597012,N,09430.9146736,W,2,12,0.49,372.953,M,-27.832,M,,0131*78";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Altitude - 372.953) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestValidGNGGA_014714()
        {
            Console.Write("Test: Valid GNGGA 014714... ");
            try
            {
                string nmea = "$GNGGA,014714.00,3617.3597246,N,09430.9146880,W,2,12,0.49,373.002,M,-27.832,M,,0131*72";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Altitude - 373.002) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestValidGNGGA_014715()
        {
            Console.Write("Test: Valid GNGGA 014715... ");
            try
            {
                string nmea = "$GNGGA,014715.00,3617.3597337,N,09430.9146890,W,2,12,0.49,373.054,M,-27.832,M,,0131*76";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Altitude - 373.054) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestValidGNGGA_014716()
        {
            Console.Write("Test: Valid GNGGA 014716... ");
            try
            {
                string nmea = "$GNGGA,014716.00,3617.3597368,N,09430.9146741,W,2,12,0.49,373.082,M,-27.832,M,,0131*77";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Altitude - 373.082) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestValidGNGGA_014717()
        {
            Console.Write("Test: Valid GNGGA 014717... ");
            try
            {
                string nmea = "$GNGGA,014717.00,3617.3597354,N,09430.9146556,W,2,12,0.51,373.145,M,-27.832,M,,0131*7E";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Altitude - 373.145) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestValidGNGGA_014718()
        {
            Console.Write("Test: Valid GNGGA 014718... ");
            try
            {
                string nmea = "$GNGGA,014718.00,3617.3597249,N,09430.9146296,W,2,12,0.51,373.182,M,-27.832,M,,0131*7C";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = Math.Abs(fix.Altitude - 373.182) < ALTITUDE_TOLERANCE &&
                              fix.RTKType == RTKTypes.None;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        // Error case tests
        private bool TestNullString()
        {
            Console.Write("Test: Null string throws exception... ");
            try
            {
                GNSSFix.ParseNMEA(null);
                Console.WriteLine("FAIL - Should have thrown exception");
                return false;
            }
            catch (NMEAParseException)
            {
                Console.WriteLine("PASS");
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Wrong exception type: {ex.GetType().Name}");
                return false;
            }
        }

        private bool TestEmptyString()
        {
            Console.Write("Test: Empty string throws exception... ");
            try
            {
                GNSSFix.ParseNMEA("");
                Console.WriteLine("FAIL - Should have thrown exception");
                return false;
            }
            catch (NMEAParseException)
            {
                Console.WriteLine("PASS");
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Wrong exception type: {ex.GetType().Name}");
                return false;
            }
        }

        private bool TestMissingDollarSign()
        {
            Console.Write("Test: Missing $ throws exception... ");
            try
            {
                GNSSFix.ParseNMEA("GNGGA,014638.00,3617.3591173,N,09430.9142232,W,2,12,0.50,372.743,M,-27.832,M,,0131*72");
                Console.WriteLine("FAIL - Should have thrown exception");
                return false;
            }
            catch (NMEAParseException)
            {
                Console.WriteLine("PASS");
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Wrong exception type: {ex.GetType().Name}");
                return false;
            }
        }

        private bool TestUnsupportedSentenceType()
        {
            Console.Write("Test: Unsupported sentence type throws exception... ");
            try
            {
                GNSSFix.ParseNMEA("$GNRMC,014638.00,A,3617.3591173,N,09430.9142232,W,0.024,,241025,,,D,V*06");
                Console.WriteLine("FAIL - Should have thrown exception");
                return false;
            }
            catch (NMEAParseException)
            {
                Console.WriteLine("PASS");
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Wrong exception type: {ex.GetType().Name}");
                return false;
            }
        }

        private bool TestIncompleteSentence()
        {
            Console.Write("Test: Incomplete sentence throws exception... ");
            try
            {
                GNSSFix.ParseNMEA("$GNGGA,014638.00,3617.3591173,N");
                Console.WriteLine("FAIL - Should have thrown exception");
                return false;
            }
            catch (NMEAParseException)
            {
                Console.WriteLine("PASS");
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Wrong exception type: {ex.GetType().Name}");
                return false;
            }
        }

        private bool TestMissingLatitude()
        {
            Console.Write("Test: Missing latitude throws exception... ");
            try
            {
                GNSSFix.ParseNMEA("$GNGGA,014638.00,,N,09430.9142232,W,2,12,0.50,372.743,M,-27.832,M,,0131*72");
                Console.WriteLine("FAIL - Should have thrown exception");
                return false;
            }
            catch (NMEAParseException)
            {
                Console.WriteLine("PASS");
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Wrong exception type: {ex.GetType().Name}");
                return false;
            }
        }

        private bool TestMissingLongitude()
        {
            Console.Write("Test: Missing longitude throws exception... ");
            try
            {
                GNSSFix.ParseNMEA("$GNGGA,014638.00,3617.3591173,N,,W,2,12,0.50,372.743,M,-27.832,M,,0131*72");
                Console.WriteLine("FAIL - Should have thrown exception");
                return false;
            }
            catch (NMEAParseException)
            {
                Console.WriteLine("PASS");
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Wrong exception type: {ex.GetType().Name}");
                return false;
            }
        }

        private bool TestInvalidLatitudeFormat()
        {
            Console.Write("Test: Invalid latitude format throws exception... ");
            try
            {
                GNSSFix.ParseNMEA("$GNGGA,014638.00,ABC,N,09430.9142232,W,2,12,0.50,372.743,M,-27.832,M,,0131*72");
                Console.WriteLine("FAIL - Should have thrown exception");
                return false;
            }
            catch (NMEAParseException)
            {
                Console.WriteLine("PASS");
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Wrong exception type: {ex.GetType().Name}");
                return false;
            }
        }

        private bool TestInvalidLongitudeFormat()
        {
            Console.Write("Test: Invalid longitude format throws exception... ");
            try
            {
                GNSSFix.ParseNMEA("$GNGGA,014638.00,3617.3591173,N,XYZ,W,2,12,0.50,372.743,M,-27.832,M,,0131*72");
                Console.WriteLine("FAIL - Should have thrown exception");
                return false;
            }
            catch (NMEAParseException)
            {
                Console.WriteLine("PASS");
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Wrong exception type: {ex.GetType().Name}");
                return false;
            }
        }

        private bool TestInvalidDirectionIndicator()
        {
            Console.Write("Test: Invalid direction indicator throws exception... ");
            try
            {
                GNSSFix.ParseNMEA("$GNGGA,014638.00,3617.3591173,X,09430.9142232,W,2,12,0.50,372.743,M,-27.832,M,,0131*72");
                Console.WriteLine("FAIL - Should have thrown exception");
                return false;
            }
            catch (NMEAParseException)
            {
                Console.WriteLine("PASS");
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Wrong exception type: {ex.GetType().Name}");
                return false;
            }
        }

        // RTK Fix tests (quality 4)
        private bool TestRTKFix_Quality4()
        {
            Console.Write("Test: RTK Fix (quality 4) - HasRTK=true, RTKType=Fix... ");
            try
            {
                // Reference: https://receiverhelp.trimble.com/alloy-gnss/en-us/NMEA-0183messages_GGA.html
                // Quality 4 = RTK Fixed, xFill
                string nmea = "$GNGGA,120000.00,3723.46587704,N,12202.26957864,W,4,12,1.2,18.893,M,-25.669,M,2.0,0031*4F";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = fix.RTKType == RTKTypes.Fix &&
                              fix.HasRTK == true;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestRTKFix_Quality4_WithXFill()
        {
            Console.Write("Test: RTK Fix (quality 4) with xFill - HasRTK=true, RTKType=Fix... ");
            try
            {
                string nmea = "$GNGGA,120000.00,3617.3591173,N,09430.9142232,W,4,12,0.50,372.743,M,-27.832,M,,0131*72";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = fix.RTKType == RTKTypes.Fix &&
                              fix.HasRTK == true &&
                              Math.Abs(fix.Latitude - 36.2893186) < POSITION_TOLERANCE &&
                              Math.Abs(fix.Longitude - (-94.5152371)) < POSITION_TOLERANCE;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        // RTK Float tests (quality 5)
        private bool TestRTKFloat_Quality5()
        {
            Console.Write("Test: RTK Float (quality 5) - HasRTK=true, RTKType=Float... ");
            try
            {
                // Reference: https://receiverhelp.trimble.com/alloy-gnss/en-us/NMEA-0183messages_GGA.html
                // Quality 5 = RTK Float, OmniSTAR XP/HP, Location RTK, RTX
                string nmea = "$GNGGA,120000.00,3723.46587704,N,12202.26957864,W,5,12,1.2,18.893,M,-25.669,M,2.0,0031*4F";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = fix.RTKType == RTKTypes.Float &&
                              fix.HasRTK == true;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestRTKFloat_Quality5_OmniSTAR()
        {
            Console.Write("Test: RTK Float (quality 5) OmniSTAR - HasRTK=true, RTKType=Float... ");
            try
            {
                string nmea = "$GNGGA,120000.00,3617.3591173,N,09430.9142232,W,5,12,0.50,372.743,M,-27.832,M,,0131*72";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = fix.RTKType == RTKTypes.Float &&
                              fix.HasRTK == true &&
                              Math.Abs(fix.Latitude - 36.2893186) < POSITION_TOLERANCE &&
                              Math.Abs(fix.Longitude - (-94.5152371)) < POSITION_TOLERANCE;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        // No RTK tests (quality 0, 1, 2, 3, 6)
        private bool TestNoRTK_Quality0_Invalid()
        {
            Console.Write("Test: No RTK (quality 0 - invalid) - HasRTK=false, RTKType=None... ");
            try
            {
                // Reference: https://receiverhelp.trimble.com/alloy-gnss/en-us/NMEA-0183messages_GGA.html
                // Quality 0 = Fix not valid
                string nmea = "$GNGGA,120000.00,3617.3591173,N,09430.9142232,W,0,12,0.50,372.743,M,-27.832,M,,0131*72";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = fix.RTKType == RTKTypes.None &&
                              fix.HasRTK == false;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestNoRTK_Quality1_GPS()
        {
            Console.Write("Test: No RTK (quality 1 - GPS) - HasRTK=false, RTKType=None... ");
            try
            {
                // Quality 1 = GPS fix
                string nmea = "$GNGGA,120000.00,3617.3591173,N,09430.9142232,W,1,12,0.50,372.743,M,-27.832,M,,0131*72";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = fix.RTKType == RTKTypes.None &&
                              fix.HasRTK == false;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestNoRTK_Quality2_DGPS()
        {
            Console.Write("Test: No RTK (quality 2 - DGPS) - HasRTK=false, RTKType=None... ");
            try
            {
                // Quality 2 = Differential GPS fix (DGNSS), SBAS, OmniSTAR VBS, Beacon, RTX in GVBS mode
                string nmea = "$GNGGA,120000.00,3617.3591173,N,09430.9142232,W,2,12,0.50,372.743,M,-27.832,M,,0131*72";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = fix.RTKType == RTKTypes.None &&
                              fix.HasRTK == false;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestNoRTK_Quality3_NotApplicable()
        {
            Console.Write("Test: No RTK (quality 3 - not applicable) - HasRTK=false, RTKType=None... ");
            try
            {
                // Quality 3 = Not applicable
                string nmea = "$GNGGA,120000.00,3617.3591173,N,09430.9142232,W,3,12,0.50,372.743,M,-27.832,M,,0131*72";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = fix.RTKType == RTKTypes.None &&
                              fix.HasRTK == false;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        private bool TestNoRTK_Quality6_INS()
        {
            Console.Write("Test: No RTK (quality 6 - INS) - HasRTK=false, RTKType=None... ");
            try
            {
                // Quality 6 = INS Dead reckoning
                string nmea = "$GNGGA,120000.00,3617.3591173,N,09430.9142232,W,6,12,0.50,372.743,M,-27.832,M,,0131*72";
                GNSSFix fix = GNSSFix.ParseNMEA(nmea);

                bool passed = fix.RTKType == RTKTypes.None &&
                              fix.HasRTK == false;

                Console.WriteLine(passed ? "PASS" : "FAIL");
                return passed;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Exception: {ex.Message}");
                return false;
            }
        }

        // Additional mangled/invalid string tests
        private bool TestOnlyDollarSign()
        {
            Console.Write("Test: Only dollar sign throws exception... ");
            try
            {
                GNSSFix.ParseNMEA("$");
                Console.WriteLine("FAIL - Should have thrown exception");
                return false;
            }
            catch (NMEAParseException)
            {
                Console.WriteLine("PASS");
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Wrong exception type: {ex.GetType().Name}");
                return false;
            }
        }

        private bool TestOnlySentenceType()
        {
            Console.Write("Test: Only sentence type throws exception... ");
            try
            {
                GNSSFix.ParseNMEA("$GNGGA");
                Console.WriteLine("FAIL - Should have thrown exception");
                return false;
            }
            catch (NMEAParseException)
            {
                Console.WriteLine("PASS");
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Wrong exception type: {ex.GetType().Name}");
                return false;
            }
        }

        private bool TestChecksumOnly()
        {
            Console.Write("Test: Checksum only throws exception... ");
            try
            {
                GNSSFix.ParseNMEA("*72");
                Console.WriteLine("FAIL - Should have thrown exception");
                return false;
            }
            catch (NMEAParseException)
            {
                Console.WriteLine("PASS");
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Wrong exception type: {ex.GetType().Name}");
                return false;
            }
        }

        private bool TestLatitudeOutOfRange()
        {
            Console.Write("Test: Latitude out of range throws exception... ");
            try
            {
                // Latitude > 90 degrees (9000.0 in DDMM format)
                GNSSFix.ParseNMEA("$GNGGA,014638.00,9000.0,N,09430.9142232,W,2,12,0.50,372.743,M,-27.832,M,,0131*72");
                Console.WriteLine("FAIL - Should have thrown exception");
                return false;
            }
            catch (NMEAParseException)
            {
                Console.WriteLine("PASS");
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Wrong exception type: {ex.GetType().Name}");
                return false;
            }
        }

        private bool TestLongitudeOutOfRange()
        {
            Console.Write("Test: Longitude out of range throws exception... ");
            try
            {
                // Longitude > 180 degrees (18000.0 in DDDMM format)
                GNSSFix.ParseNMEA("$GNGGA,014638.00,3617.3591173,N,18000.0,W,2,12,0.50,372.743,M,-27.832,M,,0131*72");
                Console.WriteLine("FAIL - Should have thrown exception");
                return false;
            }
            catch (NMEAParseException)
            {
                Console.WriteLine("PASS");
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Wrong exception type: {ex.GetType().Name}");
                return false;
            }
        }

        private bool TestLatitudeMinutesOutOfRange()
        {
            Console.Write("Test: Latitude minutes out of range throws exception... ");
            try
            {
                // Minutes >= 60 (e.g., 3617.60 would be 36 degrees 17.6 minutes, but we'll use 60+ minutes)
                GNSSFix.ParseNMEA("$GNGGA,014638.00,3660.0,N,09430.9142232,W,2,12,0.50,372.743,M,-27.832,M,,0131*72");
                Console.WriteLine("FAIL - Should have thrown exception");
                return false;
            }
            catch (NMEAParseException)
            {
                Console.WriteLine("PASS");
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Wrong exception type: {ex.GetType().Name}");
                return false;
            }
        }

        private bool TestLongitudeMinutesOutOfRange()
        {
            Console.Write("Test: Longitude minutes out of range throws exception... ");
            try
            {
                // Minutes >= 60
                GNSSFix.ParseNMEA("$GNGGA,014638.00,3617.3591173,N,09460.0,W,2,12,0.50,372.743,M,-27.832,M,,0131*72");
                Console.WriteLine("FAIL - Should have thrown exception");
                return false;
            }
            catch (NMEAParseException)
            {
                Console.WriteLine("PASS");
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Wrong exception type: {ex.GetType().Name}");
                return false;
            }
        }

        private bool TestInvalidAltitudeFormat()
        {
            Console.Write("Test: Invalid altitude format throws exception... ");
            try
            {
                GNSSFix.ParseNMEA("$GNGGA,014638.00,3617.3591173,N,09430.9142232,W,2,12,0.50,ABC,M,-27.832,M,,0131*72");
                Console.WriteLine("FAIL - Should have thrown exception");
                return false;
            }
            catch (NMEAParseException)
            {
                Console.WriteLine("PASS");
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Wrong exception type: {ex.GetType().Name}");
                return false;
            }
        }

        private bool TestInvalidQualityFormat()
        {
            Console.Write("Test: Invalid quality format throws exception... ");
            try
            {
                GNSSFix.ParseNMEA("$GNGGA,014638.00,3617.3591173,N,09430.9142232,W,XYZ,12,0.50,372.743,M,-27.832,M,,0131*72");
                Console.WriteLine("FAIL - Should have thrown exception");
                return false;
            }
            catch (NMEAParseException)
            {
                Console.WriteLine("PASS");
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Wrong exception type: {ex.GetType().Name}");
                return false;
            }
        }

        private bool TestMissingLatitudeDirection()
        {
            Console.Write("Test: Missing latitude direction throws exception... ");
            try
            {
                GNSSFix.ParseNMEA("$GNGGA,014638.00,3617.3591173,,09430.9142232,W,2,12,0.50,372.743,M,-27.832,M,,0131*72");
                Console.WriteLine("FAIL - Should have thrown exception");
                return false;
            }
            catch (NMEAParseException)
            {
                Console.WriteLine("PASS");
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Wrong exception type: {ex.GetType().Name}");
                return false;
            }
        }

        private bool TestMissingLongitudeDirection()
        {
            Console.Write("Test: Missing longitude direction throws exception... ");
            try
            {
                GNSSFix.ParseNMEA("$GNGGA,014638.00,3617.3591173,N,09430.9142232,,2,12,0.50,372.743,M,-27.832,M,,0131*72");
                Console.WriteLine("FAIL - Should have thrown exception");
                return false;
            }
            catch (NMEAParseException)
            {
                Console.WriteLine("PASS");
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Wrong exception type: {ex.GetType().Name}");
                return false;
            }
        }

        private bool TestWhitespaceOnly()
        {
            Console.Write("Test: Whitespace only throws exception... ");
            try
            {
                GNSSFix.ParseNMEA("   ");
                Console.WriteLine("FAIL - Should have thrown exception");
                return false;
            }
            catch (NMEAParseException)
            {
                Console.WriteLine("PASS");
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Wrong exception type: {ex.GetType().Name}");
                return false;
            }
        }

        private bool TestExtraCommas()
        {
            Console.Write("Test: Extra commas throws exception... ");
            try
            {
                // Missing fields due to extra commas
                GNSSFix.ParseNMEA("$GNGGA,014638.00,,,3617.3591173,N,09430.9142232,W,2,12,0.50,372.743,M,-27.832,M,,0131*72");
                Console.WriteLine("FAIL - Should have thrown exception");
                return false;
            }
            catch (NMEAParseException)
            {
                Console.WriteLine("PASS");
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Wrong exception type: {ex.GetType().Name}");
                return false;
            }
        }

        private bool TestInvalidCharactersInLatitude()
        {
            Console.Write("Test: Invalid characters in latitude throws exception... ");
            try
            {
                GNSSFix.ParseNMEA("$GNGGA,014638.00,36A7.3591173,N,09430.9142232,W,2,12,0.50,372.743,M,-27.832,M,,0131*72");
                Console.WriteLine("FAIL - Should have thrown exception");
                return false;
            }
            catch (NMEAParseException)
            {
                Console.WriteLine("PASS");
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Wrong exception type: {ex.GetType().Name}");
                return false;
            }
        }

        private bool TestInvalidCharactersInLongitude()
        {
            Console.Write("Test: Invalid characters in longitude throws exception... ");
            try
            {
                GNSSFix.ParseNMEA("$GNGGA,014638.00,3617.3591173,N,094B0.9142232,W,2,12,0.50,372.743,M,-27.832,M,,0131*72");
                Console.WriteLine("FAIL - Should have thrown exception");
                return false;
            }
            catch (NMEAParseException)
            {
                Console.WriteLine("PASS");
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAIL - Wrong exception type: {ex.GetType().Name}");
                return false;
            }
        }
    }
}

