package org.firstinspires.ftc.teamcode.robot.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterLookupTable {
    public static ShooterLookupTableInstance.VelocityLookupValue[] NORMAL_VEL_DATA_POINTS = new ShooterLookupTableInstance.VelocityLookupValue[]{
            // endpoint (prevent crashing)
            new ShooterLookupTableInstance.VelocityLookupValue(0, 395),

            new ShooterLookupTableInstance.VelocityLookupValue(27.7,405),
            new ShooterLookupTableInstance.VelocityLookupValue(45.5,420),
            new ShooterLookupTableInstance.VelocityLookupValue(68.7,490),
            new ShooterLookupTableInstance.VelocityLookupValue(80,500),
            new ShooterLookupTableInstance.VelocityLookupValue(90.7,540),
            new ShooterLookupTableInstance.VelocityLookupValue(104.4,555),
            new ShooterLookupTableInstance.VelocityLookupValue(122.7,685),
            new ShooterLookupTableInstance.VelocityLookupValue(128.7,730),
            new ShooterLookupTableInstance.VelocityLookupValue(140.2,765),
            new ShooterLookupTableInstance.VelocityLookupValue(155,780),
            new ShooterLookupTableInstance.VelocityLookupValue(165,800),

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
            new ShooterLookupTableInstance.HoodLookupValue(67,0.45),
            new ShooterLookupTableInstance.HoodLookupValue(97.7,0.65),
            new ShooterLookupTableInstance.HoodLookupValue(126,.85),
            new ShooterLookupTableInstance.HoodLookupValue(130,.87),
            new ShooterLookupTableInstance.HoodLookupValue(140,.92),
            new ShooterLookupTableInstance.HoodLookupValue(155,1.0),
            //endpoint
    };

    public static final ShooterLookupTableInstance NORMAL_TABLE = new ShooterLookupTableInstance(
            NORMAL_VEL_DATA_POINTS, NORMAL_HOOD_DATA_POINTS,
            0.0, 0.0
    );



    public static ShooterLookupTableInstance.VelocityLookupValue[] SORTED_VEL_DATA_POINTS = new ShooterLookupTableInstance.VelocityLookupValue[]{
            // endpoint (prevent crashing)
            new ShooterLookupTableInstance.VelocityLookupValue(0, 395),

            new ShooterLookupTableInstance.VelocityLookupValue(27.7,405),
            new ShooterLookupTableInstance.VelocityLookupValue(45.5,420),
            new ShooterLookupTableInstance.VelocityLookupValue(68.7,490),
            new ShooterLookupTableInstance.VelocityLookupValue(80,500),
            new ShooterLookupTableInstance.VelocityLookupValue(90.7,550),
            new ShooterLookupTableInstance.VelocityLookupValue(104.4,565),
            new ShooterLookupTableInstance.VelocityLookupValue(122.7,685),
            new ShooterLookupTableInstance.VelocityLookupValue(128.7,730),
            new ShooterLookupTableInstance.VelocityLookupValue(140.2,765),
            new ShooterLookupTableInstance.VelocityLookupValue(155,780),
            new ShooterLookupTableInstance.VelocityLookupValue(165,800),

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
            new ShooterLookupTableInstance.HoodLookupValue(67,0.45),
            new ShooterLookupTableInstance.HoodLookupValue(97.7,0.65),
            new ShooterLookupTableInstance.HoodLookupValue(126,.85),
            new ShooterLookupTableInstance.HoodLookupValue(130,.87),
            new ShooterLookupTableInstance.HoodLookupValue(140,.92),
            new ShooterLookupTableInstance.HoodLookupValue(155,1.0),
            //endpoint
    };
    public static final ShooterLookupTableInstance SORTED_TABLE = new ShooterLookupTableInstance(
            SORTED_VEL_DATA_POINTS, SORTED_HOOD_DATA_POINTS,
            -4.0, 0.0
    );
}
