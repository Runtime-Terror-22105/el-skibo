package org.firstinspires.ftc.teamcode.robot.subsystems.vision;

import java.util.HashMap;
import java.util.Map;

public class VisionConstants {
    public static class APRILTAG {
        public static Map<Integer, String> tagMap = new HashMap<Integer, String>() {{
            put(20, "BLUESCORE");
            put(21, "GPP");
            put(22, "PGP");
            put(23,"PPG");
            put(24, "REDSCORE");
        }};



        //assuming the robot is facing forward these should be relative
        /*
        top down view
        ^
        |
        |
        x (center of the robot 0,0)

         */
        public static class cameraOffset {
            static final double x = 5; //meters or whatever unit is read from the camera
            static final double y = 5;
            static final double z = 5;
            static final double pitchRad = 0;//if the camera was the head of the robot this would be like looking up/down
            static final double pitchYaw = 0; //angle difference between the forward facing vector drawn above

        }
    }
}
