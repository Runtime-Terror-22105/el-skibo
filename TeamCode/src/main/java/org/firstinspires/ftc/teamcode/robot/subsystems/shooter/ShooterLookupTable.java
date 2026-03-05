package org.firstinspires.ftc.teamcode.robot.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterLookupTable {
    public static ShooterLookupTableInstance.VelocityLookupValue[] NORMAL_VEL_DATA_POINTS = new ShooterLookupTableInstance.VelocityLookupValue[]{
            // endpoint (prevent crashing)
            new ShooterLookupTableInstance.VelocityLookupValue(0, 400),

            new ShooterLookupTableInstance.VelocityLookupValue(27.7,400),
            new ShooterLookupTableInstance.VelocityLookupValue(45.5,425),
            new ShooterLookupTableInstance.VelocityLookupValue(68.7,465),
            new ShooterLookupTableInstance.VelocityLookupValue(90.7,505),
            new ShooterLookupTableInstance.VelocityLookupValue(104.4,545),
            new ShooterLookupTableInstance.VelocityLookupValue(122.7,570),
            new ShooterLookupTableInstance.VelocityLookupValue(140.2,640),
            // other endpoint (prevent crashing)

//            new LookupValue(250, 960)
//
    };
    public static ShooterLookupTableInstance.HoodLookupValue[] NORMAL_HOOD_DATA_POINTS = new ShooterLookupTableInstance.HoodLookupValue[]{
            //endpoint
            new ShooterLookupTableInstance.HoodLookupValue(0, 0),

            new ShooterLookupTableInstance.HoodLookupValue(27.7,0),
            new ShooterLookupTableInstance.HoodLookupValue(45.5,0.25),
            new ShooterLookupTableInstance.HoodLookupValue(68.7,0.75),
            new ShooterLookupTableInstance.HoodLookupValue(90.7,0.75),
            new ShooterLookupTableInstance.HoodLookupValue(104.4,0.85),
            new ShooterLookupTableInstance.HoodLookupValue(122.7,.92),
            new ShooterLookupTableInstance.HoodLookupValue(140.2,.92),
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

    public static final ShooterLookupTableInstance NORMAL_TABLE = new ShooterLookupTableInstance(NORMAL_VEL_DATA_POINTS, NORMAL_HOOD_DATA_POINTS);



    public static ShooterLookupTableInstance.VelocityLookupValue[] SORTED_VEL_DATA_POINTS = new ShooterLookupTableInstance.VelocityLookupValue[]{
            // endpoint (prevent crashing)
            new ShooterLookupTableInstance.VelocityLookupValue(0, 405),

            new ShooterLookupTableInstance.VelocityLookupValue(27.7,405),
            new ShooterLookupTableInstance.VelocityLookupValue(45.5,430),
            new ShooterLookupTableInstance.VelocityLookupValue(68.7,470),
            new ShooterLookupTableInstance.VelocityLookupValue(90.7,510),
            new ShooterLookupTableInstance.VelocityLookupValue(104.4,550),
            new ShooterLookupTableInstance.VelocityLookupValue(122.7,575),
            new ShooterLookupTableInstance.VelocityLookupValue(140.2,645),
            // other endpoint (prevent crashing)
//
    };
    public static ShooterLookupTableInstance.HoodLookupValue[] SORTED_HOOD_DATA_POINTS = new ShooterLookupTableInstance.HoodLookupValue[]{
            //endpoint
            new ShooterLookupTableInstance.HoodLookupValue(0, 0),

            new ShooterLookupTableInstance.HoodLookupValue(27.7,0),
            new ShooterLookupTableInstance.HoodLookupValue(45.5,0.21),
            new ShooterLookupTableInstance.HoodLookupValue(68.7,0.71),
            new ShooterLookupTableInstance.HoodLookupValue(90.7,0.71),
            new ShooterLookupTableInstance.HoodLookupValue(104.4,0.81),
            new ShooterLookupTableInstance.HoodLookupValue(122.7,.88),
            new ShooterLookupTableInstance.HoodLookupValue(140.2,.88),
            //endpoint
    };
    public static final ShooterLookupTableInstance SORTED_TABLE = new ShooterLookupTableInstance(SORTED_VEL_DATA_POINTS, SORTED_HOOD_DATA_POINTS);
}
