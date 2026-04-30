using Xunit;

namespace AutoTransferWindowPlanner.Tests;

public sealed class ManeuverSeedRuleTests
{
    [Fact]
    public void EightSignMasksIncludeNormalComponentFlip()
    {
        SignMask[] masks = ManeuverSeedRules.SignMasks;

        Assert.Equal(8, masks.Length);
        Assert.Contains(masks, mask => mask.Z == -1.0);
        Assert.Contains(masks, mask => mask.X == -1.0 && mask.Y == -1.0 && mask.Z == -1.0);
    }
}
