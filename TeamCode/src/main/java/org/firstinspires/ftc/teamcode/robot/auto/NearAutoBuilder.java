package org.firstinspires.ftc.teamcode.robot.auto;

import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.AFTER_GATE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.GATE_CONTROL_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.GATE_CONTROL_POSE_2;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.GATE_INTAKE_DELAY;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.HITTING_GATE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_1_BEFORE_HORIZ_CONTROL;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_1_BEFORE_HORIZ_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_1_CONTROL;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_1_HORIZ_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_1_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_2_BEFORE_HORIZ_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_2_CONTROL;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_2_HORIZ_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_2_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_3_BEFORE_HORIZ_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_3_CONTROL;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_3_HORIZ_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_3_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_DELAY;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_SPINDEX_TIMEOUT_HORIZ;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_TIMEOUT_HORIZ;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.MAX_DRIVETRAIN_POWER_INTAKING;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.PREPARE_INTAKE_3_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.PREPARE_PUSH_GATE_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.PUSH_GATE_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.RELAXED_CONSTRAINTS;

import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.pedroPathing.FixedHeadingInterpolator;
import org.firstinspires.ftc.teamcode.robot.command.WaitForIntakeCommand;
import org.firstinspires.ftc.teamcode.robot.command.intake.SetIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.WaitForSpindexerYawCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToIntakeStateCommand;

public final class NearAutoBuilder {
    private NearAutoBuilder() {
    }

    private static PathChain intakeSpike1Path(AutoBuildState state) {
        state.lastPath = PathUtil.addPathBuilderCurve(state.robot, state.startPoseBlue, state.lastPath, INTAKE_1_CONTROL, INTAKE_1_POSE, state.mirror, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
        return state.lastPath;
    }

    private static PathChain intakeSpike2Path(AutoBuildState state) {
        state.lastPath = PathUtil.addPathBuilderCurve(state.robot, state.startPoseBlue, state.lastPath, INTAKE_2_CONTROL, INTAKE_2_POSE, state.mirror, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
        return state.lastPath;
    }

    private static PathChain prepareIntakeSpike3Path(AutoBuildState state) {
        Pose2d prepareIntakePose = PREPARE_INTAKE_3_POSE.mirror(state.mirror);
        state.lastPath = PathUtil.addPathBuilderCurve(state.robot, state.startPoseBlue, state.lastPath, INTAKE_3_CONTROL, PREPARE_INTAKE_3_POSE, state.mirror, false, false)
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(0.0, 0.4, HeadingInterpolator.tangent),
                                new HeadingInterpolator.PiecewiseNode(0.4, 0.7,
                                        FixedHeadingInterpolator.linearFromPoint(
                                                () -> state.robot.follower.getHeading(),
                                                prepareIntakePose.heading,
                                                0.4, 0.7
                                        )
                                ),
                                new HeadingInterpolator.PiecewiseNode(0.7, 1.0,
                                        HeadingInterpolator.constant(prepareIntakePose.heading)
                                )
                        )
                )
                .build();
        return state.lastPath;
    }

    private static PathChain intakeSpike3Path(AutoBuildState state) {
        state.lastPath = PathUtil.addPathBuilderLine(state.robot, state.startPoseBlue, state.lastPath, INTAKE_3_POSE, state.mirror, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
        return state.lastPath;
    }

    private static PathChain preparePushGatePath(AutoBuildState state) {
        state.lastPath = PathUtil.addPathBuilderLine(state.robot, state.lastPath, PREPARE_PUSH_GATE_POSE, state.mirror, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
        return state.lastPath;
    }

    private static PathChain pushGatePath(AutoBuildState state) {
        state.lastPath = PathUtil.addPathBuilderLine(state.robot, state.lastPath, PUSH_GATE_POSE, state.mirror, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
        return state.lastPath;
    }

    private static PathChain intakeGatePath1(AutoBuildState state) {
        state.lastPath = PathUtil.addPathBuilderCurve(state.robot, state.startPoseBlue, state.lastPath, GATE_CONTROL_POSE, HITTING_GATE, state.mirror, false, false)
                .setNoDeceleration()
                .build();
        return state.lastPath;
    }

    private static PathChain intakeGatePath2(AutoBuildState state) {
        state.lastPath = PathUtil.addPathBuilderCurve(state.robot, state.startPoseBlue, state.lastPath, GATE_CONTROL_POSE_2, AFTER_GATE, state.mirror, false, false)
                .setConstantHeadingInterpolation(AFTER_GATE.mirror(state.mirror).heading)
                .setBrakingStrength(0.6)
                .build();
        return state.lastPath;
    }

    private static Command intakeSpike1(AutoBuildState state) {
        PathChain path = intakeSpike1Path(state);
        Command followPathCommand = new FollowPathAndWaitForWallCommand(state.robot, path, true, MAX_DRIVETRAIN_POWER_INTAKING, 3.0)
                .raceWith(new WaitForIntakeCommand(state.robot));
        return new SequentialCommandGroup(
                new GoToIntakeStateCommand(state.robot),
                followPathCommand,
                new WaitForIntakeCommand(state.robot).withTimeout(INTAKE_DELAY),
                new SetIntakeSpeedCommand(state.robot.intake, 0)
        );
    }

    private static Command intakeSpike2(AutoBuildState state) {
        PathChain path = intakeSpike2Path(state);
        Command followPathCommand = new FollowPathAndWaitForWallCommand(state.robot, path, true, MAX_DRIVETRAIN_POWER_INTAKING, 18.0)
                .raceWith(new WaitForIntakeCommand(state.robot));
        return new SequentialCommandGroup(
                new GoToIntakeStateCommand(state.robot),
                followPathCommand,
                new WaitForIntakeCommand(state.robot).withTimeout(INTAKE_DELAY),
                new SetIntakeSpeedCommand(state.robot.intake, 0)
        );
    }

    private static Command intakeSpike3(AutoBuildState state) {
        return new SequentialCommandGroup(
                new GoToIntakeStateCommand(state.robot),
                new FollowPathAndWaitForWallCommand(state.robot, prepareIntakeSpike3Path(state), true, MAX_DRIVETRAIN_POWER_INTAKING, 36.0),
                new FollowPathCommand(state.robot.follower, intakeSpike3Path(state), true),
                new WaitForIntakeCommand(state.robot).withTimeout(INTAKE_DELAY),
                new SetIntakeSpeedCommand(state.robot.intake, 0)
        );
    }

    public static Command shootPreload(AutoBuildState state, ShootPathFlag... flagArr) {
        return UnsortedShootRoutines.shootPreload(state, flagArr);
    }

    public static Command intakeSpike(AutoBuildState state, int spikeNumber) {
        switch (spikeNumber) {
            case 1:
                return intakeSpike1(state);
            case 2:
                return intakeSpike2(state);
            case 3:
                return intakeSpike3(state);
            default:
                throw new IllegalArgumentException("Invalid spike number: " + spikeNumber);
        }
    }

    public static Command intakeSpikeHorizontal(AutoBuildState state, int spikeNumber) {
        // intakeBeforeHorizPath is necessary to be at the right x-value so we drive head-on into the balls
        // or else we will hit it at an angle
        Pose2d intakePose;
        Pose2d intakeBeforeHorizPose;
        PathBuilder intakeBeforeHorizPathBuilder;
        if (spikeNumber == 1) {
            intakePose = INTAKE_1_HORIZ_POSE;
            intakeBeforeHorizPose = INTAKE_1_BEFORE_HORIZ_POSE;
            intakeBeforeHorizPathBuilder = PathUtil.addPathBuilderCurve(
                    state.robot, state.startPoseBlue, state.lastPath,
                    INTAKE_1_BEFORE_HORIZ_CONTROL, intakeBeforeHorizPose, state.mirror,
                    false, false
            );
        } else if (spikeNumber == 2) {
            intakePose = INTAKE_2_HORIZ_POSE;
            intakeBeforeHorizPose = INTAKE_2_BEFORE_HORIZ_POSE;
            intakeBeforeHorizPathBuilder = PathUtil.addPathBuilderLine(
                    state.robot, state.startPoseBlue, state.lastPath,
                    intakeBeforeHorizPose, state.mirror,
                    false, false
            );
        } else if (spikeNumber == 3) {
            intakePose = INTAKE_3_HORIZ_POSE;
            intakeBeforeHorizPose = INTAKE_3_BEFORE_HORIZ_POSE;
            intakeBeforeHorizPathBuilder = PathUtil.addPathBuilderLine(
                    state.robot, state.startPoseBlue, state.lastPath,
                    intakeBeforeHorizPose, state.mirror,
                    false, false
            );
        } else {
            throw new IllegalArgumentException("Invalid spike number: " + spikeNumber);
        }

        PathChain intakeBeforeHorizPath = intakeBeforeHorizPathBuilder
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();

        state.lastPath = PathUtil.addPathBuilderLine(
                state.robot, state.startPoseBlue,
                        intakeBeforeHorizPath, intakePose,
                        state.mirror, false, false)
                .setConstantHeadingInterpolation(intakeBeforeHorizPose.mirror(state.mirror).heading)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();

//        state.lastPath = intakeBeforeHorizPathBuilder
//                .setConstraintsForLast(RELAXED_CONSTRAINTS)
//                .addPath(new BezierLine(
//                        intakeBeforeHorizPose.mirror(state.mirror).toPedro(),
//                        intakePose.mirror(state.mirror).toPedro()
//                ))
//                .setConstantHeadingInterpolation(INTAKE_1_BEFORE_HORIZ_POSE.mirror(state.mirror).heading)
//                .setConstraintsForLast(RELAXED_CONSTRAINTS)
//                .build();

//        state.lastPath = PathUtil.addPathBuilderLine(state.robot, state.startPoseBlue, state.lastPath, intakePose, state.mirror, false, false)
//                .setConstraintsForLast(RELAXED_CONSTRAINTS)
//                .build();

        return new SequentialCommandGroup(
                new FollowPathCommand(state.robot.follower, intakeBeforeHorizPath, true, MAX_DRIVETRAIN_POWER_INTAKING),
                new WaitForSpindexerYawCommand(state.robot.spindexer, intakePose.heading).withTimeout(INTAKE_SPINDEX_TIMEOUT_HORIZ),
                new FollowPathCommand(state.robot.follower, state.lastPath, true, MAX_DRIVETRAIN_POWER_INTAKING),
                new WaitForIntakeCommand(state.robot).withTimeout(INTAKE_TIMEOUT_HORIZ)
        );
    }

    public static Command shootSpike(AutoBuildState state, int spikeNumber, ShootPathFlag... flagArr) {
        return UnsortedShootRoutines.shootSpike(state, spikeNumber, flagArr);
    }

    public static Command cycleSpike(AutoBuildState state, int spikeNumber, ShootPathFlag... flags) {
        return new SequentialCommandGroup(
                intakeSpike(state, spikeNumber),
                shootSpike(state, spikeNumber, flags)
        );
    }

    public static Command pushGate(AutoBuildState state) {
        return new SequentialCommandGroup(
                new FollowPathCommand(state.robot.follower, preparePushGatePath(state), true, MAX_DRIVETRAIN_POWER_INTAKING),
                new FollowPathCommand(state.robot.follower, pushGatePath(state), true, MAX_DRIVETRAIN_POWER_INTAKING)
        );
    }

    public static Command intakeGate(AutoBuildState state) {
        return new SequentialCommandGroup(
                new FollowPathCommand(state.robot.follower, intakeGatePath1(state), true, MAX_DRIVETRAIN_POWER_INTAKING),
                new FollowPathCommand(state.robot.follower, intakeGatePath2(state), true, MAX_DRIVETRAIN_POWER_INTAKING),
                new WaitForIntakeCommand(state.robot).withTimeout(GATE_INTAKE_DELAY)
        );
    }

    public static Command shootGate(AutoBuildState state, boolean reverseIntake, ShootPathFlag... flagArr) {
        return UnsortedShootRoutines.shootGate(state, reverseIntake, flagArr);
    }

    public static Command cycleGate(AutoBuildState state, boolean reverseIntake, ShootPathFlag... flags) {
        return new SequentialCommandGroup(
                intakeGate(state),
                shootGate(state, reverseIntake, flags)
        );
    }
}

