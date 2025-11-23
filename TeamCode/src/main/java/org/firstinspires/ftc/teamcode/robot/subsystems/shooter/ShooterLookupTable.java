package org.firstinspires.ftc.teamcode.robot.subsystems.shooter;

import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.robot.subsystems.ShooterSubsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class ShooterLookupTable {
    private static final double MIN_DIST = 35.01;
    private static final double MAX_DIST = 69.967;

    private static final double CUTOFF = 50;

    private static final double SHORT_HOOD = 0.9; // rad
    private static final double FAR_HOOD = 1.0; // rad

    private static final InterpLUT SHORT_VELOCITY_LUT = new InterpLUT(); // in/s
    private static InterpLUT FAR_VELOCITY_LUT = new InterpLUT(); // in/s

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
                    new LookupValue(35, 450),
                    new LookupValue(45, 470),
                    new LookupValue(55, 490),
                    new LookupValue(60, 500),
                    new LookupValue(70, 530)
            };

    static {
//        for (int i = 0; i < DATA_POINTS.size(); i++) {
//            FAR_VELOCITY_LUT.add(DATA_POINTS.get(i).first, DATA_POINTS.get(i).second);
//        }
//        FAR_VELOCITY_LUT.createLUT();
    }

    public static ShooterSubsystem.ShooterValues get(double distanceToGoalIn) {
        // todo: temporarily putting this here so we can dashboard
        FAR_VELOCITY_LUT = new InterpLUT();
        for (LookupValue dataPoint : DATA_POINTS) {
            FAR_VELOCITY_LUT.add(dataPoint.distance, dataPoint.speed);
        }
        FAR_VELOCITY_LUT.createLUT();


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
