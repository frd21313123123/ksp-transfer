using System;

namespace AutoTransferWindowPlanner
{
    internal static class AstrodynamicsMath
    {
        internal static double CircularOrbitToHyperbolaDV(double bodyRadius, double gravParameter, double altitude, double vinf)
        {
            if (!IsFinite(bodyRadius) || !IsFinite(gravParameter) || !IsFinite(altitude) || !IsFinite(vinf) ||
                bodyRadius <= 0.0 || gravParameter <= 0.0 || vinf < 0.0)
            {
                return double.PositiveInfinity;
            }

            double radius = Math.Max(bodyRadius + altitude, bodyRadius + 1.0);
            double circular = Math.Sqrt(gravParameter / radius);
            double hyperbola = Math.Sqrt(vinf * vinf + 2.0 * gravParameter / radius);
            return Math.Max(0.0, hyperbola - circular);
        }

        internal static HohmannEstimate EstimateHohmann(double centralMu, double originOrbitRadius, double targetOrbitRadius)
        {
            if (!IsFinite(centralMu) || !IsFinite(originOrbitRadius) || !IsFinite(targetOrbitRadius) ||
                centralMu <= 0.0 || originOrbitRadius <= 0.0 || targetOrbitRadius <= 0.0)
            {
                return HohmannEstimate.Invalid;
            }

            double transferSemiMajorAxis = (originOrbitRadius + targetOrbitRadius) * 0.5;
            double originCircular = Math.Sqrt(centralMu / originOrbitRadius);
            double targetCircular = Math.Sqrt(centralMu / targetOrbitRadius);
            double transferAtOrigin = Math.Sqrt(centralMu * (2.0 / originOrbitRadius - 1.0 / transferSemiMajorAxis));
            double transferAtTarget = Math.Sqrt(centralMu * (2.0 / targetOrbitRadius - 1.0 / transferSemiMajorAxis));
            double timeOfFlight = Math.PI * Math.Sqrt(transferSemiMajorAxis * transferSemiMajorAxis * transferSemiMajorAxis / centralMu);
            double targetMeanMotion = Math.Sqrt(centralMu / (targetOrbitRadius * targetOrbitRadius * targetOrbitRadius));
            double phaseAngleDeg = NormalizeDegrees(180.0 - targetMeanMotion * timeOfFlight * 180.0 / Math.PI);

            return new HohmannEstimate
            {
                IsValid = true,
                TransferSemiMajorAxis = transferSemiMajorAxis,
                DepartureVInf = Math.Abs(transferAtOrigin - originCircular),
                ArrivalVInf = Math.Abs(targetCircular - transferAtTarget),
                TimeOfFlight = timeOfFlight,
                PhaseAngleDeg = phaseAngleDeg
            };
        }

        private static double NormalizeDegrees(double degrees)
        {
            if (!IsFinite(degrees))
            {
                return double.NaN;
            }

            degrees %= 360.0;
            if (degrees < 0.0)
            {
                degrees += 360.0;
            }

            return degrees;
        }

        private static bool IsFinite(double value)
        {
            return !double.IsNaN(value) && !double.IsInfinity(value);
        }
    }

    internal struct HohmannEstimate
    {
        internal static readonly HohmannEstimate Invalid = new HohmannEstimate { IsValid = false };

        internal bool IsValid;
        internal double TransferSemiMajorAxis;
        internal double DepartureVInf;
        internal double ArrivalVInf;
        internal double TimeOfFlight;
        internal double PhaseAngleDeg;
    }
}
