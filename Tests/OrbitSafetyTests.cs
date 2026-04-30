using AutoTransferWindowPlanner;
using Xunit;

namespace AutoTransferWindowPlanner.Tests;

public sealed class OrbitSafetyTests
{
    [Theory]
    [InlineData(0.0)]
    [InlineData(50000.0)]
    [InlineData(90000.0)]
    public void EveCaptureAltitudeBelowSafeLimitIsRejected(double altitudeMeters)
    {
        double minimum = OrbitSafety.MinimumSafeAltitude(700000.0, true, 96708.6, double.NaN);

        Assert.False(OrbitSafety.IsAltitudeSafe(altitudeMeters, minimum));
    }

    [Fact]
    public void Duna100KmIsAccepted()
    {
        double minimum = OrbitSafety.MinimumSafeAltitude(320000.0, true, 50000.0, double.NaN);

        Assert.True(OrbitSafety.IsAltitudeSafe(100000.0, minimum));
    }

    [Fact]
    public void TerrainRadiusRaisesMinimumSafeAltitude()
    {
        double minimum = OrbitSafety.MinimumSafeAltitude(200000.0, false, 0.0, 207000.0);

        Assert.Equal(8000.0, minimum);
    }
}
