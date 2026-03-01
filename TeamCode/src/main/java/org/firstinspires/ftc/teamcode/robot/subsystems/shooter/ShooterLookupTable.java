package org.firstinspires.ftc.teamcode.robot.subsystems.shooter;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.math.LinearInterpLUT;
import org.firstinspires.ftc.teamcode.robot.subsystems.ShooterSubsystem;

@Config
public class ShooterLookupTable {
    private static LinearInterpLUT FAR_VELOCITY_LUT; // in/s
    private static LinearInterpLUT HOOD_LUT;

    public static boolean debug = false;

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
            // endpoint (prevent crashing)
            new LookupValue(0, 400),

            new LookupValue(27.7,400),
            new LookupValue(45.5,425),
            new LookupValue(68.7,465),
            new LookupValue(90.7,505),
            new LookupValue(104.4,545),
            new LookupValue(122.7,570),
            new LookupValue(140.2,640),
            // other endpoint (prevent crashing)

//            new LookupValue(250, 960)
//
    };
    public static HoodLookupValue[] HOOD_DATA_POINTS = new HoodLookupValue[]{
            //endpoint
            new HoodLookupValue(0, 0),

            new HoodLookupValue(27.7,0),
            new HoodLookupValue(45.5,0.25),
            new HoodLookupValue(68.7,0.75),
            new HoodLookupValue(90.7,0.75),
            new HoodLookupValue(104.4,0.85),
            new HoodLookupValue(122.7,.92),
            new HoodLookupValue(140.2,.92),
            //endpoint

//            //endpoint
//            new HoodLookupValue(250, ShooterSubsystem.hoodAngleMax)


            // new vals
//            new HoodLookupValue(39.16, 0.7),
//            new HoodLookupValue(50.07, 0.708),
//            new HoodLookupValue(64.2, 0.74),
//            new HoodLookupValue(80.5, 0.76),
//            new HoodLookupValue(120.6, 0.9),
//            new HoodLookupValue(129.5, 0.9),

            // old vals
//            new HoodLookupValue(20.4, 0.7),
//            new HoodLookupValue(39.502, 0.735),
//            new HoodLookupValue(54.99, 0.735),
//            new HoodLookupValue(62.7, 0.8),//rad
//            new HoodLookupValue(83.3, 0.8),
//            new HoodLookupValue(87.56152811761086, 0.85),
    };

    static {
//        for (int i = 0; i < DATA_POINTS.size(); i++) {
//            FAR_VELOCITY_LUT.add(DATA_POINTS.get(i).first, DATA_POINTS.get(i).second);
//        }
//        FAR_VELOCITY_LUT.createLUT();
    }

    public static ShooterSubsystem.ShooterValues get(double distanceToGoalIn) {
        if (debug) Log.i("ShooterLookupTable", "distance to goal" +distanceToGoalIn);
        // todo: temporarily putting this here so we can dashboard
        FAR_VELOCITY_LUT = new LinearInterpLUT();
        HOOD_LUT = new LinearInterpLUT();
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
