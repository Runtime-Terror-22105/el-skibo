package org.firstinspires.ftc.teamcode.robot.subsystems.shooter;

import android.util.Log;
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

    private static final InterpLUT SHORT_VELOCITY_LUT = new InterpLUT(); // in/s
    private static InterpLUT FAR_VELOCITY_LUT; // in/s

    public static class LookupValue {
        public double distance;
        public double speed;

        public LookupValue(double d, double s) {
            this.distance = d;
            this.speed = s;
        }
    }

    public static LookupValue[] DATA_POINTS = new LookupValue[]{
            new LookupValue(35, 451),
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

    public static double get(double distanceToGoalIn) {
        // todo: temporarily putting this here so we can dashboard
        FAR_VELOCITY_LUT = new InterpLUT();
        for (LookupValue dataPoint : DATA_POINTS) {
            Log.i("ShooterLookupTable", "("+dataPoint.distance + ", "+dataPoint.speed+")");
            FAR_VELOCITY_LUT.add(dataPoint.distance, dataPoint.speed);
        }
        FAR_VELOCITY_LUT.createLUT();

        distanceToGoalIn = Math.max(MIN_DIST, Math.min(MAX_DIST, distanceToGoalIn));

        return FAR_VELOCITY_LUT.get(distanceToGoalIn);
    }
}
