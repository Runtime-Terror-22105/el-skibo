package org.firstinspires.ftc.teamcode.math;

public class Angle {
    /**
     * Do the angle optimization for swerve.
     * @return The optimized angle.
     */
    public static double optimize(double angle, double goalAngle) {
        if (Angle.canBeOptimized(angle, goalAngle)) {
            return Angle.angleWrap(angle - Math.PI);
        }
        return angle;
    }

    public static boolean canBeOptimized(double angle, double goalAngle) {
        double error = Angle.angleWrap(goalAngle - angle);
        return Math.abs(error) > Math.PI / 2;
    }

    /**
     * Change an angle to be between 0 and 360 degrees.
     * @param angle An angle, in radians.
     * @return The normalized angle.
     */
    public static double normalize(double angle) {
        // avoid pinpoint crashout issue
        if (Math.abs(angle) > 4 * Math.PI) {
            double numRevolutions = Math.floor(Math.abs(angle) / (2*Math.PI));
            angle = Math.signum(angle) * (Math.abs(angle) - (numRevolutions * (2*Math.PI)));
        }

        angle = angle % (2 * Math.PI);
        return angle < 0 ? angle + 2 * Math.PI : angle;
    }

    /**
     * Change an angle to be between 0 and 360 degrees.
     * @param angle An angle, in radians.
     * @return The normalized angle.
     */
    public static float normalize(float angle) {
        return angle % ((float) (2 * Math.PI));
    }

    /**
     * Gets the difference between two angles.
     * @param angle1 The first angle
     * @param angle2 The second angle
     * @return The difference between the two angles, in radians
     */
    public static double getdelta(double angle1, double angle2) { // function for difference between 2 angles
        double[] changes = {-2.0*Math.PI, 0.0, 2.0*Math.PI}; // add/subtract 2pi radians
        double min_delta = Math.abs(angle2 - angle1);
        for (int i=0; i < 3; i++) {
            min_delta = Math.min(min_delta, Math.abs((angle2 + changes[i]) - angle1));
        }
        return min_delta;
    }

    public static double angleWrap(double angle) {
        // avoid pinpoint crashout issue
//        if (Math.abs(angle) > 4 * Math.PI) {
//            double numRevolutions = Math.floor(Math.abs(angle) / (2*Math.PI));
//            angle = Math.signum(angle) * (Math.abs(angle) - (numRevolutions * (2*Math.PI)));
//        }
        angle = angle % (2 * Math.PI);

        if (angle > Math.PI) {
            angle -= 2 * Math.PI;
        } else if (angle <= -Math.PI) {
            angle += 2 * Math.PI;
        }

        return angle;
    }

    public static double angleWrap(float angle) {
        // avoid pinpoint crashout issue
//        if (Math.abs(angle) > 4 * Math.PI) {
//            double numRevolutions = Math.floor(Math.abs(angle) / (2*Math.PI));
//            angle = Math.signum(angle) * (Math.abs(angle) - (numRevolutions * (2*Math.PI)));
//        }
        float rotation = (float) (2 * Math.PI);
        angle = angle % rotation;

        if (angle > Math.PI) {
            angle -= rotation;
        } else if (angle <= -Math.PI) {
            angle += rotation;
        }

        return angle;
    }
}
