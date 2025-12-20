package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.math.Pose2d;

@Config
public class FieldConstants {
    public static Pose2d BLUE_GOAL_POS = new Pose2d(6, 138, 0.0);
    public static Pose2d RED_GOAL_POS = new Pose2d(138, 138, 0.0);
    public static Pose2d BLUE_START_POS_TELEOP = new Pose2d(20, 123, Math.toRadians(-37.7));
    public static Pose2d RED_START_POS_TELEOP = new Pose2d(124, 123, Math.toRadians(217.7));
    public static Pose2d BLUE_START_POS_AUTO = new Pose2d(20, 123, Math.toRadians(-37.7));
    public static Pose2d RED_START_POS_AUTO = new Pose2d(124, 123, Math.toRadians(217.7));

    public static double TILE_WIDTH = 23.6;

    public static String MOTIF_DATA_KEY = "motif";
    public static String AUTO_ENDING_DATA_KEY = "auto ending pos, tele starting pos";
    public static String SPINDEXER_POSITION_KEY = "spindexer position at end of teleop";
}
