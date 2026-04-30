using AutoTransferWindowPlanner;
using Xunit;

namespace AutoTransferWindowPlanner.Tests;

public sealed class AstrodynamicsMathTests
{
    private const double KerbolMu = 1.1723328e18;
    private const double KerbinOrbitRadius = 13599840256.0;
    private const double DunaOrbitRadius = 20726155264.0;
    private const double KerbinRadius = 600000.0;
    private const double KerbinMu = 3.5316e12;
    private const double DunaRadius = 320000.0;
    private const double DunaMu = 3.0136321e11;

    [Fact]
    public void CircularOrbitToHyperbolaDvKerbin100KmMatchesBaseline()
    {
        double dv = AstrodynamicsMath.CircularOrbitToHyperbolaDV(KerbinRadius, KerbinMu, 100000.0, 918.35);

        Assert.InRange(dv, 1055.0, 1066.0);
    }

    [Fact]
    public void KerbinToDunaHohmannBaselineIsReasonable()
    {
        HohmannEstimate hohmann = AstrodynamicsMath.EstimateHohmann(KerbolMu, KerbinOrbitRadius, DunaOrbitRadius);
        double ejectionDv = AstrodynamicsMath.CircularOrbitToHyperbolaDV(KerbinRadius, KerbinMu, 100000.0, hohmann.DepartureVInf);
        double captureDv = AstrodynamicsMath.CircularOrbitToHyperbolaDV(DunaRadius, DunaMu, 100000.0, hohmann.ArrivalVInf);

        Assert.True(hohmann.IsValid);
        Assert.InRange(hohmann.DepartureVInf, 913.0, 923.5);
        Assert.InRange(hohmann.ArrivalVInf, 820.0, 832.0);
        Assert.InRange(ejectionDv + captureDv, 1658.0, 1679.0);
        Assert.InRange(hohmann.TimeOfFlight, 6520000.0, 6530000.0);
        Assert.InRange(hohmann.PhaseAngleDeg, 43.0, 46.0);
    }
}
