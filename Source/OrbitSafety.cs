using System;

namespace AutoTransferWindowPlanner
{
    internal static class OrbitSafety
    {
        internal const double DefaultMinimumAltitude = 1000.0;
        internal const double AtmosphereMargin = 5000.0;
        internal const double TerrainMargin = 1000.0;

        internal static double MinimumSafeAltitude(
            double bodyRadius,
            bool hasAtmosphere,
            double atmosphereDepth,
            double terrainRadiusMax)
        {
            double minimum = DefaultMinimumAltitude;

            if (hasAtmosphere && IsFinite(atmosphereDepth) && atmosphereDepth > 0.0)
            {
                minimum = Math.Max(minimum, atmosphereDepth + AtmosphereMargin);
            }

            if (IsFinite(bodyRadius) && bodyRadius > 0.0 && IsFinite(terrainRadiusMax) && terrainRadiusMax > bodyRadius)
            {
                minimum = Math.Max(minimum, terrainRadiusMax - bodyRadius + TerrainMargin);
            }

            return minimum;
        }

        internal static bool IsAltitudeSafe(double altitude, double minimumSafeAltitude)
        {
            return IsFinite(altitude) && altitude >= minimumSafeAltitude;
        }

        private static bool IsFinite(double value)
        {
            return !double.IsNaN(value) && !double.IsInfinity(value);
        }
    }
}
