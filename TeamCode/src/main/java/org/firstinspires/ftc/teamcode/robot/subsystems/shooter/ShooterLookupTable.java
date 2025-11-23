package org.firstinspires.ftc.teamcode.robot.subsystems.shooter;

import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.robot.subsystems.ShooterSubsystem;

public class ShooterLookupTable {
    private static final double MIN_DIST = 35.01;
    private static final double MAX_DIST = 69.967;

    private static final double CUTOFF = 50;

    private static final double SHORT_HOOD = 0.9; // rad
    private static final double FAR_HOOD = 1.0; // rad

    private static final InterpLUT SHORT_VELOCITY_LUT = new InterpLUT(); // in/s
    private static final InterpLUT FAR_VELOCITY_LUT = new InterpLUT(); // in/s

    static {
        FAR_VELOCITY_LUT.add(35, 430);
        FAR_VELOCITY_LUT.add(45, 450);
        FAR_VELOCITY_LUT.add(55, 500);
        FAR_VELOCITY_LUT.add(60, 510);
        FAR_VELOCITY_LUT.add(70, 545);

//        SHORT_VELOCITY_LUT.createLUT();
        FAR_VELOCITY_LUT.createLUT();
    }

    public static ShooterSubsystem.ShooterValues get(double distanceToGoalIn) {
        distanceToGoalIn = Math.max(MIN_DIST, Math.min(MAX_DIST, distanceToGoalIn));
        double velocity, hood;
//        if (distanceToGoalIn < CUTOFF) {
//            velocity = SHORT_VELOCITY_LUT.get(distanceToGoalIn);
//            hood = SHORT_HOOD;
//        } else {
            velocity = FAR_VELOCITY_LUT.get(distanceToGoalIn);
            hood = FAR_HOOD;
//        }
        return new ShooterSubsystem.ShooterValues(velocity, hood);
    }
}
