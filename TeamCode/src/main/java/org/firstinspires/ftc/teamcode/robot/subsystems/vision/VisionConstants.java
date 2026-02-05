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

        //if you see two pairs at the start of the game, it's always one of these cases

        public static Map<Integer[], Integer> BlueObeliskPairs = new HashMap<Integer[], Integer>() {{
            put(new Integer[]{21,22}, 21);
            put(new Integer[]{22,23}, 22);
            put(new Integer[]{21,23},23);
        }};

        public static Map<Integer[], Integer> RedObeliskPairs = new HashMap<Integer[], Integer>() {{
            put(new Integer[]{21,23}, 21);
            put(new Integer[]{22,23}, 23);
            put(new Integer[]{21,22},22);
        }};
        // obelisk
        //     22 Δ 23
        //       21


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
