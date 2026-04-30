using AutoTransferWindowPlanner;
using Xunit;

namespace AutoTransferWindowPlanner.Tests;

public sealed class PlanningRuleTests
{
    [Fact]
    public void DirectModeUsesFirstTarget()
    {
        Assert.Equal(0, RoutePlanningRules.GetDirectTargetIndex(3));
    }

    [Fact]
    public void FinalCapturePrefersLastCaptureTarget()
    {
        bool[] captures = { false, true, false, true, false };

        int targetIndex = RoutePlanningRules.GetFinalTargetIndex(captures.Length, true, i => captures[i]);

        Assert.Equal(3, targetIndex);
    }

    [Fact]
    public void AdjacentValidationChecksNeighborPairs()
    {
        bool[] allowedPairs = { true, false };

        bool valid = RoutePlanningRules.TryValidateAdjacentPairs(3, pairIndex => allowedPairs[pairIndex], out int failedPairIndex);

        Assert.False(valid);
        Assert.Equal(1, failedPairIndex);
    }
}
