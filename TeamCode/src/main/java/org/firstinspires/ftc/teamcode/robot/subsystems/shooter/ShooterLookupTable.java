package org.firstinspires.ftc.teamcode.robot.subsystems.shooter;

import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.robot.subsystems.ShooterSubsystem;

public class ShooterLookupTable {
    // LUT from distance to hood/velocity.
    private static final InterpLUT HOOD_LUT = new InterpLUT(); // rad
    private static final InterpLUT VELOCITY_LUT = new InterpLUT(); // in/s

    private static final double MIN_DIST = 35.01;
    private static final double MAX_DIST = 73.967;

    static {
        HOOD_LUT.add(35, 1.02);
        VELOCITY_LUT.add(35, 450);

        HOOD_LUT.add(55.214, 1);
        VELOCITY_LUT.add(55.214, 488);

        HOOD_LUT.add(73.968, 1.2);
        VELOCITY_LUT.add(73.968, 500);

//        HOOD_LUT.add(87, 1.02);
//        VELOCITY_LUT.add(87, 450);

        HOOD_LUT.createLUT();
        VELOCITY_LUT.createLUT();
    }

    public static ShooterSubsystem.ShooterValues get(double distanceToGoalIn) {
        distanceToGoalIn = Math.max(MIN_DIST, Math.min(MAX_DIST, distanceToGoalIn));
        double hoodPos = HOOD_LUT.get(distanceToGoalIn);
        double velocity = VELOCITY_LUT.get(distanceToGoalIn);
        return new ShooterSubsystem.ShooterValues(velocity, hoodPos);
    }
}
