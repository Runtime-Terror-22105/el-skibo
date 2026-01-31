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
    public static Pose2d SHOOT_PRELOAD_POSE = new Pose2d(50.0, 93, Math.toRadians(225));
    public static Pose2d SHOOT_EDGE_POSE = new Pose2d(60, 83, Math.toRadians(45));
    public static Pose2d SHOOT_LAST_POSE = new Pose2d(50, 116, Math.toRadians(315));

    public static Pose2d PREPARE_INTAKE_1_POSE = new Pose2d(52.598, 85.149, Math.toRadians(180));
    public static Pose2d INTAKE_1_CONTROL = new Pose2d(58, 83, 0);
    public static Pose2d INTAKE_1_POSE = new Pose2d(25, 85.149, Math.toRadians(180));

    public static Pose2d PREPARE_INTAKE_2_POSE = new Pose2d(PREPARE_INTAKE_1_POSE.x, 60, Math.toRadians(180));
    public static Pose2d INTAKE_2_CONTROL = new Pose2d(58, 58, 0);
    public static Pose2d INTAKE_2_POSE = new Pose2d(20, 60, Math.toRadians(180));

    public static Pose2d PREPARE_INTAKE_3_POSE = new Pose2d(PREPARE_INTAKE_1_POSE.x, 37, Math.toRadians(180));
    public static Pose2d INTAKE_3_POSE = new Pose2d(20, 39, Math.toRadians(180));

    public static Pose2d GATE_CONTROL_POSE = new Pose2d(55, 61, Math.toRadians(180));
    public static Pose2d BEFORE_GATE = new Pose2d(22.542, 62.2, Math.toRadians(157));
    public static Pose2d AFTER_GATE = new Pose2d(11, 62.2, Math.toRadians(157));

    public static Pose2d PUSH_GATE_POSE = new Pose2d(23, 72.827, Math.toRadians(180));

    public static int PRE_INTAKE_DELAY = 0;
    public static int INTAKE_DELAY = 600;
    public static int GATE_INTAKE_TIMEOUT = 1500;
    public static int PRELOAD_PRE_SHOOT_DELAY = 250;
    public static int PRE_SHOOT_DELAY = 0;
    public static int SHOOT_DELAY = 0;

    public static double MAX_DRIVETRAIN_POWER_INTAKING = 0.8;

    public static PathConstraints RELAXED_CONSTRAINTS;
    static {
        RELAXED_CONSTRAINTS = Constants.pathConstraints.copy();
        RELAXED_CONSTRAINTS.setTValueConstraint(0.93);
        RELAXED_CONSTRAINTS.setVelocityConstraint(10);
        RELAXED_CONSTRAINTS.setTranslationalConstraint(5);
        RELAXED_CONSTRAINTS.setHeadingConstraint(0.07);
        RELAXED_CONSTRAINTS.setTimeoutConstraint(0);
    }
}
