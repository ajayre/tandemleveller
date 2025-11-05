using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Controller_Test_Harness
{
    public class GNSSFix
    {
        public double Longitude;
        public double Latitude;
        public double Altitude;  // meters
        public double Heading;
        public double Speed;     // kph
        public bool HasRTK;      // true for fix and float

        public GNSSFix() : this(0, 0, 0, 0, 0, false)
        {
        }

        public GNSSFix
            (
            double Longitude,
            double Latitude,
            double Altitude,
            double Heading,
            double Speed,
            bool HasRTK
            )
        {
            this.Longitude = Longitude;
            this.Latitude  = Latitude;
            this.Altitude  = Altitude;
            this.Heading   = Heading;
            this.Speed     = Speed;
            this.HasRTK    = HasRTK;
        }
    }
}
