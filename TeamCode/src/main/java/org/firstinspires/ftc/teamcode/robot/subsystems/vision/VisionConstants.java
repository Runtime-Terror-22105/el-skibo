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
    }
}
