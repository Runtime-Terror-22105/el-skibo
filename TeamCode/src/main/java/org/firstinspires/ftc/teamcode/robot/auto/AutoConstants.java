package org.firstinspires.ftc.teamcode.robot.auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.paths.PathConstraints;

import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Contains constants for auto positions on the field to reduce repetition between auto files.
 */
@Config
public class AutoConstants {
    // Unless otherwise specified, all poses are for NEAR ZONE.
    public static Pose2d SHOOT_PRELOAD_POSE = new Pose2d(53, 98, 4.2);
    public static Pose2d SHOOT_PRELOAD_FAR_POSE = new Pose2d(48, 8, Math.toRadians(180));
    public static Pose2d SHOOT_PRELOAD_FAR_POSE_ALT = new Pose2d(48, 10, Math.toRadians(125));
    public static Pose2d SHOOT_PRELOAD_HORIZ_POSE = new Pose2d(24, 115, Math.toRadians(225));
    public static Pose2d SHOOT_EDGE_POSE = new Pose2d(47, 78, Math.toRadians(-124.7));
    public static Pose2d SHOOT_EDGE_HORIZ_POSE = new Pose2d(36, 104, Math.toRadians(225));
    public static Pose2d SHOOT_LAST_POSE = new Pose2d(57, 95, Math.toRadians(-124.7));
    public static Pose2d SHOOT_FAR_POSE = new Pose2d(48-3.315, 14, Math.toRadians(180));
    public static Pose2d AUTO_FAR_ROTATED_POSE = new Pose2d(48, 8, Math.toRadians(180));
    public static Pose2d SHOOT_FAR_POSE_FORWARD_FACING = new Pose2d(48, 8, Math.toRadians(165));
//    public static Pose2d SHOOT_FAR_POSE_FORWARD_FACING = SHOOT_FAR_POSE;
    public static Pose2d SHOOT_SORTED_POSE_2 = new Pose2d(54, 80, Math.toRadians(-124.7));
    public static Pose2d SHOOT_SORTED_POSE_3 = new Pose2d(SHOOT_SORTED_POSE_2.x, SHOOT_SORTED_POSE_2.y, Math.toRadians(180));
    public static Pose2d SHOOT_SORTED_POSE_1 = new Pose2d(SHOOT_SORTED_POSE_2.x, SHOOT_SORTED_POSE_2.y, Math.toRadians(235));

    public static Pose2d INTAKE_1_BEFORE_HORIZ_CONTROL = new Pose2d(24, 111, 0);
    public static Pose2d INTAKE_1_BEFORE_HORIZ_POSE = new Pose2d(27, 97, Math.toRadians(263));
    public static Pose2d INTAKE_1_HORIZ_POSE = new Pose2d(INTAKE_1_BEFORE_HORIZ_POSE.x, 82, Math.toRadians(263));
    public static Pose2d INTAKE_2_BEFORE_HORIZ_POSE = new Pose2d(27, 78, Math.toRadians(269));
    public static Pose2d INTAKE_2_HORIZ_POSE = new Pose2d(INTAKE_1_BEFORE_HORIZ_POSE.x, 63, Math.toRadians(263));
    public static Pose2d INTAKE_3_BEFORE_HORIZ_POSE = new Pose2d(27, 54, Math.toRadians(263));
    public static Pose2d INTAKE_3_HORIZ_POSE = new Pose2d(INTAKE_1_BEFORE_HORIZ_POSE.x, 34, Math.toRadians(263));

    public static Pose2d INTAKE_1_CONTROL = new Pose2d(58, 83, 0);
    public static Pose2d SORTED_INTAKE_1_CONTROL = new Pose2d(40, 83, 0);
    public static Pose2d INTAKE_1_POSE = new Pose2d(25, 83, Math.toRadians(180));

    public static Pose2d INTAKE_2_CONTROL = new Pose2d(58, 58, 0);
    public static Pose2d INTAKE_2_POSE = new Pose2d(22, 60, Math.toRadians(150));
    public static Pose2d INTAKE_2_CONTROL_PUSH_GATE = new Pose2d(49, 53, Math.toRadians(50));
    public static Pose2d INTAKE_2_POSE_PUSH_GATE = new Pose2d(22, 65, Math.toRadians(180));

    public static Pose2d PREPARE_INTAKE_2_POSE_FAR = new Pose2d(50, 55, Math.toRadians(180));
    public static Pose2d PREPARE_INTAKE_2_CONTROL_FAR = new Pose2d(50, 44, Math.toRadians(180));

    public static Pose2d INTAKE_2_CONTROL_FAR = new Pose2d(20, 61, Math.toRadians(180));
    public static Pose2d INTAKE_2_POSE_FAR = new Pose2d(16, 61, Math.toRadians(180));


    public static Pose2d PREPARE_INTAKE_3_POSE = new Pose2d(52.598, 37, Math.toRadians(180));
    public static Pose2d INTAKE_3_CONTROL = new Pose2d(56.751, 40);
    public static Pose2d INTAKE_3_POSE = new Pose2d(18, 34, Math.toRadians(180));
    public static Pose2d PREPARE_INTAKE_3_CONTROL_FAR = new Pose2d(49-3.315, 35);
    public static Pose2d PREPARE_INTAKE_3_POSE_FAR = new Pose2d(36.42918454935622-3.315, 34, Math.toRadians(180));
    public static Pose2d INTAKE_3_POSE_FAR = new Pose2d(19-3.315, PREPARE_INTAKE_3_POSE_FAR.y, Math.toRadians(180));

    // For pushing gate after a SPIKE STRIP.
    public static Pose2d PREPARE_PUSH_GATE_POSE = new Pose2d(25, 65, Math.toRadians(180));
    public static Pose2d PUSH_GATE_POSE = new Pose2d(18.5, 70, Math.toRadians(180));

    // For CYCLING gate.
    public static Pose2d GATE_CONTROL_POSE = new Pose2d(55, 68.5);
    public static Pose2d HITTING_GATE = new Pose2d(17, 65, Math.toRadians(180));
    public static Pose2d GATE_CONTROL_POSE_2 = new Pose2d(15.9, 61.42);
    public static Pose2d AFTER_GATE = new Pose2d(11, 57, Math.toRadians(115));

    // For doing normal gate cycling
    public static Pose2d GATE_CONTROL_POSE_NORMAL = new Pose2d(40, 58);
    public static Pose2d HITTING_GATE_NORMAL = new Pose2d(10.5, 60.5,  Math.toRadians(155));

    public static Pose2d INTAKE_WALL_POSE = new Pose2d(12, 8.5, Math.toRadians(180));
    public static Pose2d INTAKE_WALL_POSE_2 = new Pose2d(17, 8, Math.toRadians(195)); // rotate into the balls a bit to avoid jams
    public static Pose2d INTAKE_WALL_VISION_POSE = new Pose2d(13-3.315, SHOOT_FAR_POSE.y, Math.toRadians(180));
    public static Pose2d INTAKE_WALL_CONTROL_POSE = new Pose2d(20, 14, Math.toRadians(180));
    public static double INTAKE_WALL_TIMEOUT_DISTANCE = 18.0;  // if the spindex walls don't go down after 18 in on the path to the wall, we stop and wait
    public static Pose2d INTAKE_WALL_CONTROL_NEAR_POSE = new Pose2d(5, 32, Math.toRadians(-90));
    public static Pose2d INTAKE_WALL_POSE_NEARAUTO = new Pose2d(8, 10, Math.toRadians(180));
    public static Pose2d INTAKE_TUNNEL_POSE = new Pose2d(11, 32, Math.toRadians(180));

    public static long FAR_BALL_CV_DETECTION_TIMEOUT = 100;
    public static long RAMP_CV_TIMEOUT = 100;
    public static long WALL_TRANSLATIONAL_CONSTRAINT = 10;

    public static Pose2d CONTROL_POSE_LONG_INTAKE = new Pose2d(17, 18, Math.toRadians(120));
    public static Pose2d START_POSE_LONG_INTAKE = new Pose2d(12, 18, Math.toRadians(120));
    public static Pose2d END_POSE_LONG_INTAKE = new Pose2d(12, 40, Math.toRadians(120));

    public static int WAIT_TIMEOUT_MOTIF = 5000;
    public static int INTAKE_DELAY = 300;
    public static int GATE_INTAKE_DELAY = 900;
    public static int WALL_INTAKE_DELAY = 200;
    public static int PRELOAD_PRE_SHOOT_DELAY = 0;
    public static int PRELOAD_FAR_PRE_SHOOT_SPINUP_TIMEOUT = 2000;
    public static int PRE_SHOOT_DELAY = 670;
    public static int SHOOT_DELAY = 0;
    public static int INTAKE_SPINDEX_TIMEOUT_HORIZ = 500;
    public static int INTAKE_TIMEOUT_HORIZ = 200;
    public static int REVERSE_INTAKE_GATE_DELAY = 250;
    public static int SORTED_SHOOT_DELAY = 0;
    public static int NORMAL_SHOOT_DELAY = 0;

    public static double SHOOT_BRAKING_STRENGTH = 0.8;
    public static double SORTED_BRAKING_STRENGTH = 0.5;

    // Distance (in inches) from the target shoot pose for which early shoot begins
    // shooting
    public static double EARLY_SHOOT_DISTANCE = 15.0;

    public static double MAX_DRIVETRAIN_POWER = 1.0;
    public static double MAX_DRIVETRAIN_POWER_INTAKING = 1.0;

    public static long TIME_TO_WAIT_FOR_INTAKE_TO_GO_UP_BEFORE_CAMERA_USAGE = 450;

    public static int FLLYWHEEL_SPIN_UP_TIMEOUT_MS = 1500;

    public static double ANGLE_NEAR_TURRET = 4.85;

    public static PathConstraints RELAXED_CONSTRAINTS;
    static {
        RELAXED_CONSTRAINTS = Constants.pathConstraints.copy();
        RELAXED_CONSTRAINTS.setTValueConstraint(0.95);
        RELAXED_CONSTRAINTS.setVelocityConstraint(10);
        RELAXED_CONSTRAINTS.setTranslationalConstraint(5);
        RELAXED_CONSTRAINTS.setHeadingConstraint(0.07);
        RELAXED_CONSTRAINTS.setTimeoutConstraint(0);
    }
}
