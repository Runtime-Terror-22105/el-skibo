package org.firstinspires.ftc.teamcode.robot.subsystems.shooter;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.robot.subsystems.ShooterSubsystem;

@Config
public class ShooterLookupTable {
    private static InterpLUT FAR_VELOCITY_LUT; // in/s
    private static InterpLUT HOOD_LUT;

    public static class LookupValue {
        public double distance;
        public double speed;

        public LookupValue(double d, double s) {
            this.distance = d;
            this.speed = s;
        }
    }

    public static class HoodLookupValue {
        public double distance;
        public double rad;

        public HoodLookupValue(double d, double r) {
            this.distance = d;
            this.rad = r;
        }

    }

    public static LookupValue[] VEL_DATA_POINTS = new LookupValue[]{
            new LookupValue(0, 600),
            new LookupValue(20.4, 580),
            new LookupValue(39.502, 610),
            new LookupValue(54.99, 660),
            new LookupValue(62.7, 665),
            new LookupValue(83.3, 700),
            new LookupValue(87.56152811761086, 720),
            new LookupValue(250, 800)

    };
    public static HoodLookupValue[] HOOD_DATA_POINTS = new HoodLookupValue[]{
            new HoodLookupValue(0, 0.7),
            new HoodLookupValue(20.4, 0.7),
            new HoodLookupValue(39.502, 0.735),
            new HoodLookupValue(54.99, 0.735),
            new HoodLookupValue(62.7, 0.8),//rad
            new HoodLookupValue(83.3, 0.8),
            new HoodLookupValue(87.56152811761086, 0.85),
            new HoodLookupValue(250, 0.9)
    };

    static {
//        for (int i = 0; i < DATA_POINTS.size(); i++) {
//            FAR_VELOCITY_LUT.add(DATA_POINTS.get(i).first, DATA_POINTS.get(i).second);
//        }
//        FAR_VELOCITY_LUT.createLUT();
    }

    public static ShooterSubsystem.ShooterValues get(double distanceToGoalIn) {
        Log.i("shooter", "distance to goal" +distanceToGoalIn);
        // todo: temporarily putting this here so we can dashboard
        FAR_VELOCITY_LUT = new InterpLUT();
        HOOD_LUT = new InterpLUT();
        for (LookupValue dataPoint : VEL_DATA_POINTS) {
            FAR_VELOCITY_LUT.add(dataPoint.distance, dataPoint.speed);
        }
        FAR_VELOCITY_LUT.createLUT();

        for (HoodLookupValue dataPoint : HOOD_DATA_POINTS) {
            HOOD_LUT.add(dataPoint.distance, dataPoint.rad);
        }
        HOOD_LUT.createLUT();

        double rad;
        double velocity;

        velocity = FAR_VELOCITY_LUT.get(
                Math.max(VEL_DATA_POINTS[0].distance,
                        Math.min(VEL_DATA_POINTS[VEL_DATA_POINTS.length - 1].distance,
                                distanceToGoalIn))
        );
        rad = HOOD_LUT.get(
                Math.max(HOOD_DATA_POINTS[0].distance,
                        Math.min(HOOD_DATA_POINTS[HOOD_DATA_POINTS.length - 1].distance,
                                distanceToGoalIn))
        );

        return new ShooterSubsystem.ShooterValues(velocity, rad);
        //return new ShooterSubsystem.ShooterValues(0,0);
    }
}
