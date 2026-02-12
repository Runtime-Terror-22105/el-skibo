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
    public static Pose2d SHOOT_PRELOAD_POSE = new Pose2d(45, 95, Math.toRadians(225));
    public static Pose2d SHOOT_EDGE_POSE = new Pose2d(45, 95, Math.toRadians(225));
    public static Pose2d SHOOT_LAST_POSE = new Pose2d(50, 116, Math.toRadians(315));

    public static Pose2d INTAKE_1_CONTROL = new Pose2d(58, 83, 0);
    public static Pose2d INTAKE_1_POSE = new Pose2d(22, 85.149, Math.toRadians(180));

    public static Pose2d INTAKE_2_CONTROL = new Pose2d(58, 58, 0);
    public static Pose2d INTAKE_2_POSE = new Pose2d(17, 60, Math.toRadians(180));

    public static Pose2d PREPARE_INTAKE_3_POSE = new Pose2d(52.598, 37, Math.toRadians(180));
    public static Pose2d INTAKE_3_CONTROL = new Pose2d(56.751, 45.668);
    public static Pose2d INTAKE_3_POSE = new Pose2d(17, 39, Math.toRadians(180));

    // For pushing gate after a SPIKE STRIP.
    public static Pose2d BEFORE_PUSH_GATE_POSE = new Pose2d(25, 65, Math.toRadians(180));
    public static Pose2d PUSH_GATE_POSE = new Pose2d(18.5, 70, Math.toRadians(180));

    // For CYCLING gate.
    public static Pose2d GATE_CONTROL_POSE = new Pose2d(55, 59.5, Math.toRadians(180));
    public static Pose2d AFTER_GATE = new Pose2d(12, 62, Math.toRadians(163));

    public static int INTAKE_DELAY = 600;
    public static int GATE_INTAKE_DELAY = 1500;
    public static int PRELOAD_PRE_SHOOT_DELAY = 250;
    public static int PRE_SHOOT_DELAY = 0;
    public static int SHOOT_DELAY = 0;

    public static double MAX_DRIVETRAIN_POWER = 1.0;
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
