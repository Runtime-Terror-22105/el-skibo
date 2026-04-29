package org.firstinspires.ftc.teamcode.robot.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterLookupTable {
    public static ShooterLookupTableInstance.VelocityLookupValue[] NORMAL_VEL_DATA_POINTS = new ShooterLookupTableInstance.VelocityLookupValue[]{
            // endpoint (prevent crashing)
            new ShooterLookupTableInstance.VelocityLookupValue(0, 250),

            new ShooterLookupTableInstance.VelocityLookupValue(27.7,330),
            new ShooterLookupTableInstance.VelocityLookupValue(45.5,350),
            new ShooterLookupTableInstance.VelocityLookupValue(68.7,338),
            new ShooterLookupTableInstance.VelocityLookupValue(80,375),
            new ShooterLookupTableInstance.VelocityLookupValue(90.7,415),
            new ShooterLookupTableInstance.VelocityLookupValue(104.4,410),
            new ShooterLookupTableInstance.VelocityLookupValue(122.7,505),
            new ShooterLookupTableInstance.VelocityLookupValue(128.7,520),
            new ShooterLookupTableInstance.VelocityLookupValue(140.2,535),
            new ShooterLookupTableInstance.VelocityLookupValue(155,550),
            new ShooterLookupTableInstance.VelocityLookupValue(165,570),

            // other endpoint (prevent crashing)

//            new LookupValue(250, 960)
//
    };
    public static ShooterLookupTableInstance.HoodLookupValue[] NORMAL_HOOD_DATA_POINTS = new ShooterLookupTableInstance.HoodLookupValue[]{
            //endpoint
            new ShooterLookupTableInstance.HoodLookupValue(0, 0.05),

            new ShooterLookupTableInstance.HoodLookupValue(37.7,0.05),
            new ShooterLookupTableInstance.HoodLookupValue(47,0.25),
            new ShooterLookupTableInstance.HoodLookupValue(54,0.4),
            new ShooterLookupTableInstance.HoodLookupValue(67,0.5),
            new ShooterLookupTableInstance.HoodLookupValue(80,0.6),
            new ShooterLookupTableInstance.HoodLookupValue(97.7,0.65),
            new ShooterLookupTableInstance.HoodLookupValue(126,.95),
            new ShooterLookupTableInstance.HoodLookupValue(130,1.0),
            new ShooterLookupTableInstance.HoodLookupValue(140,.9),
            new ShooterLookupTableInstance.HoodLookupValue(155,1.0),
            //endpoint
    };

    public static final ShooterLookupTableInstance NORMAL_TABLE = new ShooterLookupTableInstance(
            NORMAL_VEL_DATA_POINTS, NORMAL_HOOD_DATA_POINTS,
            0.0, 0.0
    );



    public static ShooterLookupTableInstance.VelocityLookupValue[] SORTED_VEL_DATA_POINTS = new ShooterLookupTableInstance.VelocityLookupValue[]{
            // endpoint (prevent crashing)
            new ShooterLookupTableInstance.VelocityLookupValue(0, 250),

            new ShooterLookupTableInstance.VelocityLookupValue(27.7,330),
            new ShooterLookupTableInstance.VelocityLookupValue(45.5,365),
            new ShooterLookupTableInstance.VelocityLookupValue(68.7,358),
            new ShooterLookupTableInstance.VelocityLookupValue(80,435),
            new ShooterLookupTableInstance.VelocityLookupValue(90.7,455),
            new ShooterLookupTableInstance.VelocityLookupValue(104.4,455),
            new ShooterLookupTableInstance.VelocityLookupValue(122.7,597),
            new ShooterLookupTableInstance.VelocityLookupValue(128.7,537),
            new ShooterLookupTableInstance.VelocityLookupValue(140.2,537),
            new ShooterLookupTableInstance.VelocityLookupValue(155,553),
            new ShooterLookupTableInstance.VelocityLookupValue(165,573),

            // other endpoint (prevent crashing)

//            new LookupValue(250, 960)
//
//
    };
    public static ShooterLookupTableInstance.HoodLookupValue[] SORTED_HOOD_DATA_POINTS = new ShooterLookupTableInstance.HoodLookupValue[]{
            //endpoint
            new ShooterLookupTableInstance.HoodLookupValue(0, 0.05),

            new ShooterLookupTableInstance.HoodLookupValue(37.7,0.05),
            new ShooterLookupTableInstance.HoodLookupValue(47,0.25),
            new ShooterLookupTableInstance.HoodLookupValue(54,0.4),
            new ShooterLookupTableInstance.HoodLookupValue(67,0.5),
            new ShooterLookupTableInstance.HoodLookupValue(80,0.6),
            new ShooterLookupTableInstance.HoodLookupValue(97.7,0.65),
            new ShooterLookupTableInstance.HoodLookupValue(126,.8),
            new ShooterLookupTableInstance.HoodLookupValue(130,.82),
            new ShooterLookupTableInstance.HoodLookupValue(140,.87),
            new ShooterLookupTableInstance.HoodLookupValue(155,1.0),
            //endpoint
    };
    public static final ShooterLookupTableInstance SORTED_TABLE = new ShooterLookupTableInstance(
            SORTED_VEL_DATA_POINTS, SORTED_HOOD_DATA_POINTS,
            -4.0, 0.0
    );
}
