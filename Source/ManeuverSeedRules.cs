namespace AutoTransferWindowPlanner
{
    internal static class ManeuverSeedRules
    {
        internal static readonly SignMask[] SignMasks =
        {
            new SignMask(1.0, 1.0, 1.0),
            new SignMask(-1.0, 1.0, 1.0),
            new SignMask(1.0, -1.0, 1.0),
            new SignMask(1.0, 1.0, -1.0),
            new SignMask(-1.0, -1.0, 1.0),
            new SignMask(-1.0, 1.0, -1.0),
            new SignMask(1.0, -1.0, -1.0),
            new SignMask(-1.0, -1.0, -1.0)
        };
    }

    internal struct SignMask
    {
        internal SignMask(double x, double y, double z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        internal double X;
        internal double Y;
        internal double Z;
    }
}
