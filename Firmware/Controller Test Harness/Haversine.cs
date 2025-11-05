using System;

namespace Controller
{
    /// <summary>
    /// Haversine formula implementation for calculating distances and bearings between GPS coordinates.
    /// Converted from C++ implementation.
    /// </summary>
    public static class Haversine
    {
        // Constants matching the C++ implementation
        // const double EARTH_RADIUS = 6371000.00; //radius of earth in metres (commented out in original)
        public const double EARTH_RADIUS = 6378137.00; //radius of earth in metres
        public const double toDegrees = 57.295779; //180/PI
        public const double toRadians = 3.1415926535897932384626433832795 / 180.0;

        /// <summary>
        /// Calculates the distance between two GPS coordinates using the Haversine formula.
        /// Returns distance in metres.
        /// </summary>
        /// <param name="from_lat1">Latitude of the first point in decimal degrees</param>
        /// <param name="from_lon1">Longitude of the first point in decimal degrees</param>
        /// <param name="to_lat2">Latitude of the second point in decimal degrees</param>
        /// <param name="to_lon2">Longitude of the second point in decimal degrees</param>
        /// <returns>Distance in metres</returns>
        public static double Distance(double from_lat1, double from_lon1, double to_lat2, double to_lon2)
        {
            //This is a haversine based distance calculation formula

            //This portion converts the current and destination GPS coords from decDegrees to Radians
            double lonR1 = from_lon1 * toRadians;
            double lonR2 = to_lon2 * toRadians;
            double latR1 = from_lat1 * toRadians;
            double latR2 = to_lat2 * toRadians;

            //This portion calculates the differences for the Radian latitudes and longitudes and saves them to variables
            double dlon = lonR2 - lonR1;
            double dlat = latR2 - latR1;

            //This portion is the Haversine Formula for distance between two points. Returned value is in metres
            // sq(sin(dlat * 0.5)) is equivalent to sin(dlat * 0.5) * sin(dlat * 0.5)
            double a = (Math.Sin(dlat * 0.5) * Math.Sin(dlat * 0.5)) + 
                       Math.Cos(latR1) * Math.Cos(latR2) * (Math.Sin(dlon * 0.5) * Math.Sin(dlon * 0.5));
            double e = 2 * Math.Atan2(Math.Sqrt(a), Math.Sqrt(1 - a));

            return EARTH_RADIUS * e;
        }

        /// <summary>
        /// Calculates the bearing between two GPS coordinates.
        /// Returns bearing in radians.
        /// </summary>
        /// <param name="from_lat1">Latitude of the first point in decimal degrees</param>
        /// <param name="from_lon1">Longitude of the first point in decimal degrees</param>
        /// <param name="to_lat2">Latitude of the second point in decimal degrees</param>
        /// <param name="to_lon2">Longitude of the second point in decimal degrees</param>
        /// <returns>Bearing in radians</returns>
        public static double Bearing(double from_lat1, double from_lon1, double to_lat2, double to_lon2)
        {
            double latR1 = from_lat1 * toRadians;
            double latR2 = to_lat2 * toRadians;
            double lonR1 = from_lon1 * toRadians;
            double lonR2 = to_lon2 * toRadians;

            //This portion is the Haversine Formula for required bearing between current 
            //location and destination. Returned value is in radians
            double y = Math.Cos(latR2) * Math.Sin(lonR2 - lonR1); //calculate y
            double x = Math.Cos(latR1) * Math.Sin(latR2) - Math.Sin(latR1) * Math.Cos(latR2) * Math.Cos(lonR2 - lonR1); //calculate x

            return Math.Atan2(y, x); //return atan2 result for bearing. Result at this point is in Radians
        }

        /// <summary>
        /// Calculates the bearing between two GPS coordinates in degrees.
        /// Note: This function preserves the original C++ implementation's parameter order,
        /// which appears to swap lat/lon compared to the Bearing function.
        /// </summary>
        /// <param name="from_lat1">Latitude of the first point in decimal degrees</param>
        /// <param name="from_lon1">Longitude of the first point in decimal degrees</param>
        /// <param name="to_lat2">Latitude of the second point in decimal degrees</param>
        /// <param name="to_lon2">Longitude of the second point in decimal degrees</param>
        /// <returns>Bearing in degrees</returns>
        public static double BearingDegrees(double from_lat1, double from_lon1, double to_lat2, double to_lon2)
        {
            // Preserving the exact same bug/behavior from the C++ code:
            // bearing(lon1, lat1, lon2, lat2) - parameters are swapped
            return Bearing(from_lon1, from_lat1, to_lon2, to_lat2) * toDegrees;
        }

        /// <summary>
        /// Moves a GPS coordinate by a specified distance and heading.
        /// Modifies the lat and lon parameters by reference.
        /// </summary>
        /// <param name="lat">Latitude in decimal degrees (modified by reference)</param>
        /// <param name="lon">Longitude in decimal degrees (modified by reference)</param>
        /// <param name="heading">Heading in degrees</param>
        /// <param name="distance">Distance in metres</param>
        public static void MoveDistanceBearing(ref double lat, ref double lon, double heading, double distance)
        {
            double offset = distance / EARTH_RADIUS;
            double latr = lat * toRadians; // RADIANS(lat)
            double lonr = lon * toRadians; // RADIANS(lon)

            double lat1sin = Math.Sin(latr);

            double lat1cos = Math.Cos(latr);
            double distcos = Math.Cos(offset);
            double distsin = Math.Sin(offset);

            heading = heading * toRadians; // RADIANS(heading)

            double newLat = Math.Asin(lat1sin * distcos +
                                      lat1cos * distsin * Math.Cos(heading));
            double newLon = lonr + Math.Atan2(Math.Sin(heading) * distsin * lat1cos,
                                              distcos - lat1sin * Math.Sin(newLat));

            lat = newLat * toDegrees; // DEGREES(lat)
            lon = newLon * toDegrees; // DEGREES(lon)
        }
    }
}

