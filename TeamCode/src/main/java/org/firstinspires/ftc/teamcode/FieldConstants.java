package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.math.Pose2d;

@Config
public class FieldConstants {
    public static Pose2d BLUE_GOAL_POS = new Pose2d(6, 138, 0.0);
    public static Pose2d RED_GOAL_POS = new Pose2d(138, 138, 0.0);

    // We do not store red start poses, as these are calculated by mirroring the blue
    // start poses.
    public static Pose2d BLUE_START_POS_NEAR = new Pose2d(20, 123, Math.toRadians(-132));
//    public static Pose2d RED_START_POS_NEAR = BLUE_START_POS_NEAR.mirror();
    public static Pose2d BLUE_START_POS_FAR = new Pose2d(48, 8, Math.toRadians(180));
//    public static Pose2d RED_START_POS_FAR = new Pose2d(96, 0, 1D / 2D * Math.PI);

    public static String MOTIF_DATA_KEY = "motif";
    public static String AUTO_ENDING_DATA_KEY = "auto ending pos, tele starting pos";
    public static String SPINDEXER_POSITION_KEY = "spindexer position at end of teleop";
    public static String TEAM_COLOR_KEY = "team color";
    public static String RED_KEY = "red";
    public static String BLUE_KEY = "blue";
    public static String TELEOP_ENDING_KEY = "teleop ending position";
}