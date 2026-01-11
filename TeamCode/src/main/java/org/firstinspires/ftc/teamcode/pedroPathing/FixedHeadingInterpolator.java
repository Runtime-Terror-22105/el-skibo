package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathPoint;

public class FixedHeadingInterpolator {
    public static HeadingInterpolator linear(double startHeadingRad, double endHeadingRad, double startT, double endT) {
        startHeadingRad = MathFunctions.normalizeAngle(startHeadingRad);
        endHeadingRad = MathFunctions.normalizeAngle(endHeadingRad);
        double finalStartHeadingRad = startHeadingRad;
        double finalEndHeadingRad = endHeadingRad;

        return closestPoint -> {
            double clampedStartT = MathFunctions.clamp(startT, 0, 0.9999);
            double clampedEndT = MathFunctions.clamp(endT, 0.0001, 1);
            double t = Math.max(Math.min((closestPoint.tValue - clampedStartT) / (clampedEndT - clampedStartT), 1.0), 0.0);
            double deltaHeading = MathFunctions.getTurnDirection(finalStartHeadingRad, finalEndHeadingRad) * MathFunctions.getSmallestAngleDifference(finalEndHeadingRad, finalStartHeadingRad);
            return MathFunctions.normalizeAngle(finalStartHeadingRad + deltaHeading * t);
        };
    }

    public static HeadingInterpolator linearFromPoint(HeadingInterpolator.FutureDouble startHeadingRad, double endHeadingRad, double startT, double endT) {
        return new HeadingInterpolator() {
            double start = Double.NaN;
            final double end = endHeadingRad;
            boolean initialized = false;

            @Override
            public double interpolate(PathPoint closestPoint) {
                return linear(start, end, startT, endT).interpolate(closestPoint);
            }

            @Override
            public void init() {
                if (!initialized) {
                    start = MathFunctions.normalizeAngle(startHeadingRad.get());
                    initialized = true;
                }
            }
        };
    }
}
