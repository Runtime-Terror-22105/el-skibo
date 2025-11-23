package org.firstinspires.ftc.teamcode.robot.subsystems.shooter;

//import static org.firstinspires.ftc.teamcode.robot.subsystems.shooter.ShooterLookupTable.FAR_VELOCITY_LUT;

import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.robot.subsystems.ShooterSubsystem;

import java.util.HashMap;
import java.util.Map;

public class HardCodedLookup {
    private static final double MIN_DIST = 35.01;
    private static final double MAX_DIST = 69.967;

    private static final double CUTOFF = 50;

    private static final double SHORT_HOOD = 0.9; // rad
    private static final double FAR_HOOD = 1.0; // rad

    private static final InterpLUT SHORT_VELOCITY_LUT = new InterpLUT(); // in/s
//    private static final InterpLUT FAR_VELOCITY_LUT = new InterpLUT(); // in/s

    public static Map<Double,Double> mapping = new HashMap<>();

    static {

        mapping.put(35.0, 370.0);
        mapping.put(45.0, 400.0);
        mapping.put(55.0, 420.0);
        mapping.put(60.0, 450.0);
        mapping.put(70.0, 500.0);

    }

    public static ShooterSubsystem.ShooterValues get(double distanceToGoalIn) {
        distanceToGoalIn = Math.max(MIN_DIST, Math.min(MAX_DIST, distanceToGoalIn));
        double velocity, hood;
//        if (distanceToGoalIn < CUTOFF) {
//            velocity = SHORT_VELOCITY_LUT.get(distanceToGoalIn);
//            hood = SHORT_HOOD;
//        } else {
            velocity = mapping.get(distanceToGoalIn);
            hood = FAR_HOOD;
//        }
        return new ShooterSubsystem.ShooterValues(velocity, hood);
    }
}
