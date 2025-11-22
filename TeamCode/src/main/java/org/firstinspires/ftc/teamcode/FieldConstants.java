package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.math.Pose2d;

public class FieldConstants {
    public static Pose2d BLUE_GOAL_POS = new Pose2d(6, 138, 0.0);
    public static Pose2d RED_GOAL_POS = new Pose2d(138, 138, 0.0);
    public static Pose BLUE_START_POS_TELEOP = new Pose(20, 123, Math.toRadians(52.3-90));
    public static Pose RED_START_POS_TELEOP = new Pose(124, 123,  Math.toRadians(-225));
    public static Pose BLUE_START_POS_AUTO = new Pose(20, 123, (25D/18D)*Math.PI);
    public static Pose RED_START_POS_AUTO = new Pose(124, 123, (30D/18D)*Math.PI);
    public static double TILE_WIDTH = 23.6;
}
