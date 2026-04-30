using System;

namespace AutoTransferWindowPlanner
{
    internal static class RoutePlanningRules
    {
        internal static int GetDirectTargetIndex(int targetCount)
        {
            return targetCount > 0 ? 0 : -1;
        }

        internal static int GetFinalTargetIndex(int targetCount, bool preferCaptureFinal, Func<int, bool> isCapture)
        {
            if (targetCount <= 0)
            {
                return -1;
            }

            if (preferCaptureFinal && isCapture != null && targetCount > 1)
            {
                for (int i = targetCount - 1; i >= 0; i--)
                {
                    if (isCapture(i))
                    {
                        return i;
                    }
                }
            }

            return targetCount - 1;
        }

        internal static bool TryValidateAdjacentPairs(int nodeCount, Func<int, bool> canConnectPair, out int failedPairIndex)
        {
            failedPairIndex = -1;
            if (nodeCount <= 1)
            {
                return false;
            }

            if (canConnectPair == null)
            {
                failedPairIndex = 0;
                return false;
            }

            for (int i = 0; i < nodeCount - 1; i++)
            {
                if (!canConnectPair(i))
                {
                    failedPairIndex = i;
                    return false;
                }
            }

            return true;
        }
    }
}
