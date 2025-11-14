using Controller;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Controller
{
    public class GNSSVector
    {
        public double TrackTrueDeg;      // deg (track made good relative to true north)
        public double TrackMagneticDeg;  // deg (track made good relative to magnetic north)
        public double Speedkph;          // kph (speed over ground)

        public double SpeedMph
        {
            get
            {
                return Speedkph * 0.621371;
            }
        }

        public GNSSVector() : this(0, 0, 0)
        {
        }

        public GNSSVector
            (
            double TrackTrueDeg,
            double TrackMagneticDeg,
            double Speedkph
            )
        {
            this.TrackTrueDeg = TrackTrueDeg;
            this.TrackMagneticDeg = TrackMagneticDeg;
            this.Speedkph = Speedkph;
        }

        /// <summary>
        /// Parses an NMEA-0183 sentence and returns a GNSSVector object.
        /// Currently supports $GPVTG and $GNVTG sentences.
        /// </summary>
        /// <param name="nmeaSentence">The NMEA sentence to parse (e.g., "$GPVTG,140.88,T,,M,8.04,N,14.89,K,D*05")</param>
        /// <returns>A GNSSVector object populated with data from the NMEA sentence</returns>
        /// <exception cref="NMEAParseException">Thrown when the sentence is malformed, corrupted, incomplete, or unsupported</exception>
        public static GNSSVector ParseNMEA(string nmeaSentence)
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

            // Check for VTG sentence (both $GPVTG and $GNVTG)
            if (fields[0].StartsWith("$GPVTG") || fields[0].StartsWith("$GNVTG"))
            {
                return ParseVTG(fields, nmeaSentence);
            }

            // Unknown sentence type
            throw new NMEAParseException($"Unsupported NMEA sentence type: {fields[0]}");
        }

        /// <summary>
        /// Parses a $GPVTG or $GNVTG sentence.
        /// Format: $GPVTG,trackTrue,T,trackMagnetic,M,speedKnots,N,speedKph,K,mode*checksum
        /// Reference: https://receiverhelp.trimble.com/alloy-gnss/en-us/NMEA-0183messages_VTG.html
        /// </summary>
        private static GNSSVector ParseVTG(string[] fields, string originalSentence)
        {
            // Validate minimum required fields (10 fields: 0-9)
            // Field 0: Message ID
            // Field 1: Track made good (degrees true)
            // Field 2: T indicator
            // Field 3: Track made good (degrees magnetic)
            // Field 4: M indicator
            // Field 5: Speed, in knots
            // Field 6: N indicator
            // Field 7: Speed over ground in kilometers/hour (kph)
            // Field 8: K indicator
            // Field 9: Mode indicator
            if (fields.Length < 10)
            {
                throw new NMEAParseException($"VTG sentence has insufficient fields. Expected at least 10, got {fields.Length}. Sentence: {originalSentence}");
            }

            // Parse track made good (degrees true) - field 1
            double trackTrueDeg = 0.0;
            if (!string.IsNullOrEmpty(fields[1]))
            {
                if (!double.TryParse(fields[1], out trackTrueDeg))
                {
                    throw new NMEAParseException($"Failed to parse track made good (true) from field '{fields[1]}' (field 1)");
                }
            }

            // Validate T indicator - field 2 (should be 'T' if track true is present)
            if (!string.IsNullOrEmpty(fields[1]) && !string.IsNullOrEmpty(fields[2]))
            {
                if (fields[2].ToUpper() != "T")
                {
                    throw new NMEAParseException($"Invalid track true indicator '{fields[2]}' (field 2). Expected 'T'");
                }
            }

            // Parse track made good (degrees magnetic) - field 3
            double trackMagneticDeg = 0.0;
            if (!string.IsNullOrEmpty(fields[3]))
            {
                if (!double.TryParse(fields[3], out trackMagneticDeg))
                {
                    throw new NMEAParseException($"Failed to parse track made good (magnetic) from field '{fields[3]}' (field 3)");
                }
            }

            // Validate M indicator - field 4 (should be 'M' if track magnetic is present)
            if (!string.IsNullOrEmpty(fields[3]) && !string.IsNullOrEmpty(fields[4]))
            {
                if (fields[4].ToUpper() != "M")
                {
                    throw new NMEAParseException($"Invalid track magnetic indicator '{fields[4]}' (field 4). Expected 'M'");
                }
            }

            // Parse speed over ground in kilometers/hour (kph) - field 7
            double speedKph = 0.0;
            if (!string.IsNullOrEmpty(fields[7]))
            {
                if (!double.TryParse(fields[7], out speedKph))
                {
                    throw new NMEAParseException($"Failed to parse speed over ground (kph) from field '{fields[7]}' (field 7)");
                }
            }

            // Validate K indicator - field 8 (should be 'K' if speed kph is present)
            if (!string.IsNullOrEmpty(fields[7]) && !string.IsNullOrEmpty(fields[8]))
            {
                if (fields[8].ToUpper() != "K")
                {
                    throw new NMEAParseException($"Invalid speed kph indicator '{fields[8]}' (field 8). Expected 'K'");
                }
            }

            return new GNSSVector(trackTrueDeg, trackMagneticDeg, speedKph);
        }
    }
}

