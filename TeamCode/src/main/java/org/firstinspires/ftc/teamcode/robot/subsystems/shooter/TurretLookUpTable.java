package org.firstinspires.ftc.teamcode.robot.subsystems.shooter;

import org.firstinspires.ftc.teamcode.math.LinearInterpLUT;

public class TurretLookUpTable {

    TurretLookUpTable(){}

    public static class TurretDataPoint{
        public double angle;
        public double pos;
        public TurretDataPoint(double angle, double pos){
            this.angle = angle;
            this.pos = pos;

        }
    }

    public static TurretDataPoint[] RIGHT_DATA = new TurretDataPoint[]

    {
        new TurretDataPoint(1D / 2D * Math.PI, 0.49),
        new TurretDataPoint(1D / 2D * Math.PI, 0.49)
    };

    public static TurretDataPoint[] LEFT_DATA = new TurretDataPoint[]

        {
                new TurretDataPoint(1D / 2D * Math.PI, 0.49),
                new TurretDataPoint(1D / 2D * Math.PI, 0.49)
        };

    public static double get_left(double angle){
        LinearInterpLUT RIGHT_LUT = new LinearInterpLUT();
        for (TurretDataPoint point : RIGHT_DATA){
            RIGHT_LUT.add(point.angle, point.pos);


        }
        return 0.0;

    }





}
