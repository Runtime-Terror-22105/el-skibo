package org.firstinspires.ftc.teamcode.robot.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.robot.subsystems.ShooterSubsystem;

@Config
public class ShooterLookupTable {
    private static final double MIN_DIST = 35.01;
    private static final double MAX_DIST = 69.967;

    private static final double CUTOFF = 50;


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
            new LookupValue(35, 451),
            new LookupValue(45, 470),
            new LookupValue(55, 490),
            new LookupValue(60, 500),
            new LookupValue(70, 530)
    };
    public static HoodLookupValue[] HOOD_DATA_POINTS = new HoodLookupValue[]{
            new HoodLookupValue(35, 451), //rad
            new HoodLookupValue(36, 451),
            new HoodLookupValue(37, 451),
            new HoodLookupValue(38, 451),
    };

    static {
//        for (int i = 0; i < DATA_POINTS.size(); i++) {
//            FAR_VELOCITY_LUT.add(DATA_POINTS.get(i).first, DATA_POINTS.get(i).second);
//        }
//        FAR_VELOCITY_LUT.createLUT();
    }

    public static ShooterSubsystem.ShooterValues get(double distanceToGoalIn) {
        // todo: temporarily putting this here so we can dashboard
//        FAR_VELOCITY_LUT = new InterpLUT();
//        HOOD_LUT = new InterpLUT();
//        for (LookupValue dataPoint : VEL_DATA_POINTS) {
//            FAR_VELOCITY_LUT.add(dataPoint.distance, dataPoint.speed);
//        }
//        FAR_VELOCITY_LUT.createLUT();
//
//        for (HoodLookupValue dataPoint : HOOD_DATA_POINTS) {
//            HOOD_LUT.add(dataPoint.distance, dataPoint.rad);
//        }
//        HOOD_LUT.createLUT();
//
//
//        distanceToGoalIn = Math.max(MIN_DIST, Math.min(MAX_DIST, distanceToGoalIn));
//        double rad;
//        double velocity;
//
//        velocity = FAR_VELOCITY_LUT.get(distanceToGoalIn);
//        rad = HOOD_LUT.get(distanceToGoalIn);
//
//        return new ShooterSubsystem.ShooterValues(velocity, rad);
        return new ShooterSubsystem.ShooterValues(0,0);
    }
}
