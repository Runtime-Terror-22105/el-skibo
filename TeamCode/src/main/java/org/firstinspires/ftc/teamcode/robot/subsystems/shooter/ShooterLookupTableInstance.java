package org.firstinspires.ftc.teamcode.robot.subsystems.shooter;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.math.LinearInterpLUT;
import org.firstinspires.ftc.teamcode.robot.subsystems.ShooterSubsystem;

public class ShooterLookupTableInstance {
    public static class VelocityLookupValue {
        public double distance;
        public double speed;

        public VelocityLookupValue(double d, double s) {
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

    private VelocityLookupValue[] velocityPoints;
    private HoodLookupValue[] hoodPoints;
    private LinearInterpLUT velocityLut; // in/s
    private LinearInterpLUT hoodLut;

    public static boolean DEBUG = false;

    public ShooterLookupTableInstance(VelocityLookupValue[] velocityPoints, HoodLookupValue[] hoodPoints) {
        this.velocityPoints = velocityPoints;
        this.hoodPoints = hoodPoints;
    }

    private void createLUTs() {
        velocityLut = new LinearInterpLUT();
        hoodLut = new LinearInterpLUT();
        for (VelocityLookupValue dataPoint : velocityPoints) {
            velocityLut.add(dataPoint.distance, dataPoint.speed);
        }
        velocityLut.createLUT();

        for (HoodLookupValue dataPoint : hoodPoints) {
            hoodLut.add(dataPoint.distance, dataPoint.rad);
        }
        hoodLut.createLUT();
    }

    public ShooterSubsystem.ShooterValues get(double distanceToGoalIn) {
        if (DEBUG) Log.i("ShooterLookupTable", "distance to goal" +distanceToGoalIn);

        createLUTs();

        double velocity = velocityLut.get(
                Math.max(velocityPoints[0].distance,
                        Math.min(velocityPoints[velocityPoints.length - 1].distance,
                                distanceToGoalIn))
        );
        double rad = hoodLut.get(
                Math.max(hoodPoints[0].distance,
                        Math.min(hoodPoints[hoodPoints.length - 1].distance,
                                distanceToGoalIn))
        );

        return new ShooterSubsystem.ShooterValues(velocity, rad);
    }
}
