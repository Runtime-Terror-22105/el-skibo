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
    public static Pose2d SHOOT_PRELOAD_HORIZ_POSE = new Pose2d(24, 115, Math.toRadians(225));
    public static Pose2d SHOOT_EDGE_POSE = new Pose2d(53, 98, Math.toRadians(-124.7));
    public static Pose2d SHOOT_EDGE_HORIZ_POSE = new Pose2d(36, 104, Math.toRadians(225));
    public static Pose2d SHOOT_LAST_POSE = new Pose2d(50, 116, Math.toRadians(315));
    public static Pose2d SHOOT_FAR_POSE = new Pose2d(51, 16, Math.toRadians(195));

    public static Pose2d INTAKE_BEFORE_HORIZ_CONTROL = new Pose2d(21, 116, 0);
    public static Pose2d INTAKE_BEFORE_HORIZ_POSE = new Pose2d(21, 102, Math.toRadians(263));
    public static Pose2d INTAKE_1_HORIZ_POSE = new Pose2d(21, 82, Math.toRadians(263));
    public static Pose2d INTAKE_2_HORIZ_POSE = new Pose2d(21, 58, Math.toRadians(263));
    public static Pose2d INTAKE_3_HORIZ_POSE = new Pose2d(21, 34, Math.toRadians(263));

    public static Pose2d INTAKE_1_CONTROL = new Pose2d(58, 83, 0);
    public static Pose2d INTAKE_1_POSE = new Pose2d(22, 85.149, Math.toRadians(180));

    public static Pose2d INTAKE_2_CONTROL = new Pose2d(58, 58, 0);
    public static Pose2d INTAKE_2_POSE = new Pose2d(17, 60, Math.toRadians(180));

    public static Pose2d PREPARE_INTAKE_3_POSE = new Pose2d(52.598, 37, Math.toRadians(180));
    public static Pose2d INTAKE_3_CONTROL = new Pose2d(56.751, 45.668);
    public static Pose2d INTAKE_3_CONTROL_FAR = new Pose2d(56, 39);
    public static Pose2d INTAKE_3_POSE = new Pose2d(17, 39, Math.toRadians(180));
    public static Pose2d INTAKE_3_POSE_FAR = new Pose2d(17, 39, Math.toRadians(200));

    // For pushing gate after a SPIKE STRIP.
    public static Pose2d PREPARE_PUSH_GATE_POSE = new Pose2d(25, 65, Math.toRadians(180));
    public static Pose2d PUSH_GATE_POSE = new Pose2d(18.5, 70, Math.toRadians(180));

    // For CYCLING gate.
    public static Pose2d GATE_CONTROL_POSE = new Pose2d(55, 68.5, Math.toRadians(180));
    public static Pose2d HITTING_GATE = new Pose2d(15, 68.77, Math.toRadians(180));
    public static Pose2d GATE_CONTROL_POSE_2 = new Pose2d(12.3, 55.5);
    public static Pose2d AFTER_GATE = new Pose2d(8.95, 53.7, Math.toRadians(120));
    public static Pose2d AFTER_GATE_OLD = new Pose2d(10, 64, Math.toRadians(155));

    public static Pose2d INTAKE_WALL_POSE = new Pose2d(11, 8, Math.toRadians(180));
    public static Pose2d INTAKE_TUNNEL_POSE = new Pose2d(11, 32, Math.toRadians(180));
    public static Pose2d VISION_POSE = new Pose2d(42, 18, Math.toRadians(180));

    public static Pose2d CONTROL_POSE_LONG_INTAKE = new Pose2d(17, 18, Math.toRadians(120));
    public static Pose2d START_POSE_LONG_INTAKE = new Pose2d(12, 18, Math.toRadians(120));
    public static Pose2d END_POSE_LONG_INTAKE = new Pose2d(12, 40, Math.toRadians(120));

    public static int WAIT_TIMEOUT_MOTIF = 5000;
    public static int INTAKE_DELAY = 600;
    public static int GATE_INTAKE_DELAY = 1200;
    public static int WALL_INTAKE_DELAY = 250;
    public static int PRELOAD_PRE_SHOOT_DELAY = 250;
    public static int PRELOAD_FAR_PRE_SHOOT_DELAY = 1500;
    public static int PRE_SHOOT_DELAY = 0;
    public static int SHOOT_DELAY = 0;
    public static int INTAKE_DELAY_HORIZ = 1000;
    public static int REVERSE_INTAKE_GATE_DELAY = 250;
    public static int SORTED_SHOOT_DELAY = 500;

    public static double SHOOT_BRAKING_STRENGTH = 0.8;
    public static double SORTED_BRAKING_STRENGTH = 0.5;

    // Distance (in inches) from the target shoot pose for which early shoot begins
    // shooting
    public static double EARLY_SHOOT_DISTANCE = 7.0;

    public static double MAX_DRIVETRAIN_POWER = 1.0;
    public static double MAX_DRIVETRAIN_POWER_INTAKING = 1.0;

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
