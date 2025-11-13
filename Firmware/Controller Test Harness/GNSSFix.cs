using Controller;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Controller_Test_Harness
{
    /// <summary>
    /// Exception thrown when an NMEA sentence is malformed, corrupted, or incomplete.
    /// </summary>
    public class NMEAParseException : Exception
    {
        public NMEAParseException(string message) : base(message) { }
        public NMEAParseException(string message, Exception innerException) : base(message, innerException) { }
    }

    public enum RTKTypes
    {
        None,
        Fix,
        Float
    }

    public class GNSSFix
    {
        public double Longitude; // deg
        public double Latitude;  // deg
        public double Altitude;  // meters
        public double Heading;   // deg
        public double Speed;     // kph
        public RTKTypes RTKType;   // RTK type: None, Fix, or Float

        /// <summary>
        /// Returns true if RTKType is Fix or Float, false otherwise.
        /// </summary>
        public bool HasRTK
        {
            get { return RTKType == RTKTypes.Fix || RTKType == RTKTypes.Float; }
        }

        public GNSSFix() : this(0, 0, 0, 0, 0, RTKTypes.None)
        {
        }

        public GNSSFix
            (
            double Longitude,
            double Latitude,
            double Altitude,
            double Heading,
            double Speed,
            RTKTypes RTKType
            )
        {
            this.Longitude = Longitude;
            this.Latitude  = Latitude;
            this.Altitude  = Altitude;
            this.Heading   = Heading;
            this.Speed     = Speed;
            this.RTKType   = RTKType;
        }

        /// <summary>
        /// Parses an NMEA-0183 sentence and returns a GNSSFix object.
        /// Currently supports $GNGGA sentences.
        /// </summary>
        /// <param name="nmeaSentence">The NMEA sentence to parse (e.g., "$GNGGA,014638.00,3617.3591173,N,09430.9142232,W,2,12,0.50,372.743,M,-27.832,M,,0131*72")</param>
        /// <returns>A GNSSFix object populated with data from the NMEA sentence</returns>
        /// <exception cref="NMEAParseException">Thrown when the sentence is malformed, corrupted, incomplete, or unsupported</exception>
        public static GNSSFix ParseNMEA(string nmeaSentence)
        {
            if (string.IsNullOrWhiteSpace(nmeaSentence))
            {
                throw new NMEAParseException("NMEA sentence is null or empty");
            }

            // Validate sentence starts with $
            if (!nmeaSentence.StartsWith("$"))
            {
                throw new NMEAParseException($"NMEA sentence must start with '$': {nmeaSentence}");
            }

            // Remove checksum if present (everything after *)
            string sentence = nmeaSentence;
            int checksumIndex = sentence.IndexOf('*');
            if (checksumIndex >= 0)
            {
                sentence = sentence.Substring(0, checksumIndex);
            }

            // Validate sentence is not empty after removing checksum
            if (string.IsNullOrWhiteSpace(sentence))
            {
                throw new NMEAParseException("NMEA sentence is empty after removing checksum");
            }

            // Split by comma
            string[] fields = sentence.Split(',');

            // Validate we have at least the sentence type
            if (fields.Length == 0 || string.IsNullOrEmpty(fields[0]))
            {
                throw new NMEAParseException("NMEA sentence has no sentence type identifier");
            }

            // Check for GNGGA sentence
            if (fields[0].StartsWith("$GNGGA"))
            {
                return ParseGNGGA(fields, nmeaSentence);
            }

            // Unknown sentence type
            throw new NMEAParseException($"Unsupported NMEA sentence type: {fields[0]}");
        }

        /// <summary>
        /// Parses a $GNGGA sentence.
        /// Format: $GNGGA,time,lat,N/S,lon,E/W,quality,numSV,HDOP,alt,M,sep,M,diffAge,diffStation*checksum
        /// </summary>
        private static GNSSFix ParseGNGGA(string[] fields, string originalSentence)
        {
            // Validate minimum required fields (15 fields: 0-14)
            if (fields.Length < 15)
            {
                throw new NMEAParseException($"GNGGA sentence has insufficient fields. Expected at least 15, got {fields.Length}. Sentence: {originalSentence}");
            }

            // Validate required fields are present
            if (string.IsNullOrEmpty(fields[2]))
            {
                throw new NMEAParseException("GNGGA sentence missing latitude field (field 2)");
            }
            if (string.IsNullOrEmpty(fields[3]))
            {
                throw new NMEAParseException("GNGGA sentence missing latitude direction (N/S) field (field 3)");
            }
            if (string.IsNullOrEmpty(fields[4]))
            {
                throw new NMEAParseException("GNGGA sentence missing longitude field (field 4)");
            }
            if (string.IsNullOrEmpty(fields[5]))
            {
                throw new NMEAParseException("GNGGA sentence missing longitude direction (E/W) field (field 5)");
            }

            // Parse latitude (format: DDMM.MMMMMM)
            double latitude;
            try
            {
                latitude = ParseLatitude(fields[2], fields[3]);
            }
            catch (Exception ex)
            {
                throw new NMEAParseException($"Failed to parse latitude from fields '{fields[2]}' and '{fields[3]}'", ex);
            }

            // Parse longitude (format: DDDMM.MMMMMM)
            double longitude;
            try
            {
                longitude = ParseLongitude(fields[4], fields[5]);
            }
            catch (Exception ex)
            {
                throw new NMEAParseException($"Failed to parse longitude from fields '{fields[4]}' and '{fields[5]}'", ex);
            }

            // Parse altitude (field 9, meters)
            double altitude = 0.0;
            if (!string.IsNullOrEmpty(fields[9]))
            {
                if (!double.TryParse(fields[9], out altitude))
                {
                    throw new NMEAParseException($"Failed to parse altitude from field '{fields[9]}' (field 9)");
                }
            }

            // Parse quality to determine RTKType
            // Quality: 0=invalid, 1=GPS, 2=DGPS (not RTK), 4=RTK Fixed, 5=RTK Float
            // Reference: https://receiverhelp.trimble.com/alloy-gnss/en-us/NMEA-0183messages_GGA.html
            RTKTypes rtkType = RTKTypes.None;
            if (!string.IsNullOrEmpty(fields[6]))
            {
                int quality;
                if (!int.TryParse(fields[6], out quality))
                {
                    throw new NMEAParseException($"Failed to parse quality indicator from field '{fields[6]}' (field 6)");
                }
                
                if (quality == 4)
                {
                    rtkType = RTKTypes.Fix;
                }
                else if (quality == 5)
                {
                    rtkType = RTKTypes.Float;
                }
                // else: quality is 0, 1, 2, 3, or 6 - not RTK, so defaults remain
            }

            // GNGGA doesn't contain heading or speed, so they remain 0
            return new GNSSFix(longitude, latitude, altitude, 0.0, 0.0, rtkType);
        }

        /// <summary>
        /// Parses latitude from NMEA format (DDMM.MMMMMM) to decimal degrees.
        /// </summary>
        /// <exception cref="NMEAParseException">Thrown when latitude data is invalid or malformed</exception>
        private static double ParseLatitude(string latString, string nsIndicator)
        {
            if (string.IsNullOrEmpty(latString))
            {
                throw new NMEAParseException("Latitude string is null or empty");
            }
            if (string.IsNullOrEmpty(nsIndicator))
            {
                throw new NMEAParseException("Latitude direction indicator (N/S) is null or empty");
            }

            if (!double.TryParse(latString, out double latValue))
            {
                throw new NMEAParseException($"Failed to parse latitude value '{latString}' as a number");
            }

            // Validate latitude format (should be DDMM.MMMMMM, so at least 4 digits before decimal)
            if (latValue < 0 || latValue >= 9000)
            {
                throw new NMEAParseException($"Latitude value '{latString}' is out of valid range (0-8999.999999)");
            }

            // Extract degrees and minutes
            // Format: DDMM.MMMMMM
            int degrees = (int)(latValue / 100);
            double minutes = latValue - (degrees * 100);

            // Validate degrees are in valid range (0-89)
            if (degrees < 0 || degrees > 89)
            {
                throw new NMEAParseException($"Latitude degrees '{degrees}' is out of valid range (0-89)");
            }

            // Validate minutes are in valid range (0-59.999999)
            if (minutes < 0 || minutes >= 60)
            {
                throw new NMEAParseException($"Latitude minutes '{minutes}' is out of valid range (0-59.999999)");
            }

            // Convert to decimal degrees
            double decimalDegrees = degrees + (minutes / 60.0);

            // Apply N/S indicator
            string upperIndicator = nsIndicator.ToUpper();
            if (upperIndicator != "N" && upperIndicator != "S")
            {
                throw new NMEAParseException($"Invalid latitude direction indicator '{nsIndicator}'. Must be 'N' or 'S'");
            }

            if (upperIndicator == "S")
            {
                decimalDegrees = -decimalDegrees;
            }

            return decimalDegrees;
        }

        /// <summary>
        /// Parses longitude from NMEA format (DDDMM.MMMMMM) to decimal degrees.
        /// </summary>
        /// <exception cref="NMEAParseException">Thrown when longitude data is invalid or malformed</exception>
        private static double ParseLongitude(string lonString, string ewIndicator)
        {
            if (string.IsNullOrEmpty(lonString))
            {
                throw new NMEAParseException("Longitude string is null or empty");
            }
            if (string.IsNullOrEmpty(ewIndicator))
            {
                throw new NMEAParseException("Longitude direction indicator (E/W) is null or empty");
            }

            if (!double.TryParse(lonString, out double lonValue))
            {
                throw new NMEAParseException($"Failed to parse longitude value '{lonString}' as a number");
            }

            // Validate longitude format (should be DDDMM.MMMMMM, so at least 5 digits before decimal)
            if (lonValue < 0 || lonValue >= 18000)
            {
                throw new NMEAParseException($"Longitude value '{lonString}' is out of valid range (0-17999.999999)");
            }

            // Extract degrees and minutes
            // Format: DDDMM.MMMMMM
            int degrees = (int)(lonValue / 100);
            double minutes = lonValue - (degrees * 100);

            // Validate degrees are in valid range (0-179)
            if (degrees < 0 || degrees > 179)
            {
                throw new NMEAParseException($"Longitude degrees '{degrees}' is out of valid range (0-179)");
            }

            // Validate minutes are in valid range (0-59.999999)
            if (minutes < 0 || minutes >= 60)
            {
                throw new NMEAParseException($"Longitude minutes '{minutes}' is out of valid range (0-59.999999)");
            }

            // Convert to decimal degrees
            double decimalDegrees = degrees + (minutes / 60.0);

            // Apply E/W indicator
            string upperIndicator = ewIndicator.ToUpper();
            if (upperIndicator != "E" && upperIndicator != "W")
            {
                throw new NMEAParseException($"Invalid longitude direction indicator '{ewIndicator}'. Must be 'E' or 'W'");
            }

            if (upperIndicator == "W")
            {
                decimalDegrees = -decimalDegrees;
            }

            return decimalDegrees;
        }

        /// <summary>
        /// Gets the distance from this fix to another fix
        /// </summary>
        /// <param name="Fix">Fix to measure to</param>
        /// <returns>Distance in meters</returns>
        public double DistanceTo
            (
            GNSSFix Fix
            )
        {
            return Haversine.Distance(this.Latitude, this.Longitude, Fix.Latitude, Fix.Longitude);
        }
    }
}
