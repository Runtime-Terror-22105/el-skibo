package org.firstinspires.ftc.teamcode.robot.subsystems.shooter;

//import static org.firstinspires.ftc.teamcode.robot.subsystems.shooter.ShooterLookupTable.FAR_VELOCITY_LUT;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.robot.subsystems.ShooterSubsystem;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

@Config
public class HardCodedLookup {
    private static final double MIN_DIST = 35.01;
    private static final double MAX_DIST = 69.967;


    private static final double FAR_HOOD = 1.0; // rad

//    private static final InterpLUT SHORT_VELOCITY_LUT = new InterpLUT(); // in/s
//    private static final InterpLUT FAR_VELOCITY_LUT = new InterpLUT(); // in/s


    public static class LookupValue {
        public double distance;
        public double speed;

        public LookupValue(double d, double s) {
            this.distance = d;
            this.speed = s;
        }
    }

    public static LookupValue[] DATA_POINTS = new LookupValue[]
            {

                    new LookupValue(35.0, 370.0),
                    new LookupValue(45.0, 400.0),
                    new LookupValue(55.0, 420.0),
                    new LookupValue(60.0, 450.0),
                    new LookupValue(70.0, 500.0),
            };

    public static double get(double distanceToGoalIn) {
        // todo: temporarily putting this here so we can dashboard
        Map<Double,Double> mapping = new HashMap<>();
        for (LookupValue dataPoint : DATA_POINTS) {
            mapping.put(dataPoint.distance, dataPoint.speed);
        }


        distanceToGoalIn = Math.max(MIN_DIST, Math.min(MAX_DIST, distanceToGoalIn));
        double velocity;
//        if (distanceToGoalIn < CUTOFF) {
//            velocity = SHORT_VELOCITY_LUT.get(distanceToGoalIn);
//            hood = SHORT_HOOD;
//        } else {
        velocity = mapping.get(distanceToGoalIn);
//        }
        return velocity;
    }
}