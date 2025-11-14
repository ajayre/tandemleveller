using System;

namespace Controller
{
    /// <summary>
    /// Exception thrown when an NMEA sentence is malformed, corrupted, or incomplete.
    /// </summary>
    public class NMEAParseException : Exception
    {
        public NMEAParseException(string message) : base(message) { }
        public NMEAParseException(string message, Exception innerException) : base(message, innerException) { }
    }
}

