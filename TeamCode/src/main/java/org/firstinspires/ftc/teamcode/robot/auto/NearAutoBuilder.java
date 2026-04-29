package org.firstinspires.ftc.teamcode.robot.auto;

import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.AFTER_GATE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.GATE_CONTROL_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.GATE_CONTROL_POSE_2;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.GATE_CONTROL_POSE_NORMAL;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.GATE_INTAKE_DELAY;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.HITTING_GATE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.HITTING_GATE_NORMAL;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_1_BEFORE_HORIZ_CONTROL;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_1_BEFORE_HORIZ_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_1_CONTROL;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_1_HORIZ_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_1_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_2_BEFORE_HORIZ_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_2_CONTROL;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_2_CONTROL_PUSH_GATE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_2_HORIZ_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_2_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_2_POSE_PUSH_GATE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_3_BEFORE_HORIZ_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_3_CONTROL;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_3_HORIZ_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_3_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_DELAY;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_TIMEOUT_HORIZ;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_WALL_CONTROL_NEAR_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_WALL_POSE_2;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.MAX_DRIVETRAIN_POWER_INTAKING;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.PREPARE_PUSH_GATE_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.PUSH_GATE_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.RELAXED_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SORTED_INTAKE_1_CONTROL;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.WALL_INTAKE_DELAY;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.WALL_TRANSLATIONAL_CONSTRAINT;
import static org.firstinspires.ftc.teamcode.robot.auto.ShootPathFlag.FIRST_WALL_SORTED;
import static org.firstinspires.ftc.teamcode.robot.auto.ShootPathFlag.LONG_GATE_PAUSE;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.command.WaitForIntakeCommand;
import org.firstinspires.ftc.teamcode.robot.command.intake.SetIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToIntakeStateCommand;
import org.firstinspires.ftc.teamcode.util.ArrayUtil;
import org.firstinspires.ftc.teamcode.util.BallColor;

import java.util.EnumSet;

public final class NearAutoBuilder {
    private NearAutoBuilder() {
    }

    private static PathChain intakeSpike1Path(AutoBuildState state, EnumSet<ShootPathFlag> flags) {
        Pose2d control = flags.contains(ShootPathFlag.SORTING) ? SORTED_INTAKE_1_CONTROL: INTAKE_1_CONTROL;
        state.lastPath = PathUtil.addPathBuilderCurve(state.robot, state.startPoseBlue, state.lastPath, control, INTAKE_1_POSE, state.mirror, true, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
        return state.lastPath;
    }

    private static PathChain intakeSpike2Path(AutoBuildState state) {
        state.lastPath = PathUtil.addPathBuilderCurve(state.robot, state.startPoseBlue, state.lastPath, INTAKE_2_CONTROL, INTAKE_2_POSE, state.mirror, true, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
        return state.lastPath;
    }

    private static PathChain intakeSpike2PathPushGate(AutoBuildState state) {
        state.lastPath = PathUtil.addPathBuilderCurve(state.robot, state.startPoseBlue, state.lastPath, INTAKE_2_CONTROL_PUSH_GATE, INTAKE_2_POSE_PUSH_GATE, state.mirror, false, false)
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0d, .7d,
                                HeadingInterpolator.constant(INTAKE_2_CONTROL_PUSH_GATE.mirror(state.mirror).heading)
                        ),
                        new HeadingInterpolator.PiecewiseNode(.7d, 1d,
                            HeadingInterpolator.linear(INTAKE_2_CONTROL_PUSH_GATE.mirror(state.mirror).heading, INTAKE_2_POSE_PUSH_GATE.mirror(state.mirror).heading)
                        )
                ))
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
        return state.lastPath;
    }

    private static PathChain intakeSpike3Path(AutoBuildState state) {
        state.lastPath = PathUtil.addPathBuilderCurve(state.robot, state.startPoseBlue, state.lastPath, INTAKE_3_CONTROL, INTAKE_3_POSE, state.mirror, true, false)
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

    private static Command intakeSpikeFollowingPath(AutoBuildState state, PathChain path, double wallTimeoutDistance) {
        Command followPathCommand = new FollowPathAndWaitForWallCommand(state.robot, path, true, MAX_DRIVETRAIN_POWER_INTAKING, wallTimeoutDistance)
                .raceWith(new WaitForIntakeCommand(state.robot));
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new GoToIntakeStateCommand(state.robot),
                    followPathCommand
                ),
                new WaitForIntakeCommand(state.robot).withTimeout(INTAKE_DELAY),
                new SetIntakeSpeedCommand(state.robot.intake, 0)
        );
    }

    private static Command intakeSpikeFollowingPath(AutoBuildState state, PathChain path) {
        return intakeSpikeFollowingPath(state, path, 12.0);
    }

    private static Command intakeSpike1(AutoBuildState state, EnumSet<ShootPathFlag> flags) {
        PathChain path = intakeSpike1Path(state, flags);
        return intakeSpikeFollowingPath(state, path);
    }

    private static Command intakeSpike2(AutoBuildState state) {
        PathChain path = intakeSpike2Path(state);
        return intakeSpikeFollowingPath(state, path);
    }

    public static Command intakeSpike2AndPushGate(AutoBuildState state) {
        return intakeSpike2AndPushGate(state, 3.0);
    }

    public static Command intakeSpike2AndPushGate(AutoBuildState state, double wallTimeoutDistance) {
        PathChain path = intakeSpike2PathPushGate(state);
        return intakeSpikeFollowingPath(state, path, wallTimeoutDistance);
    }

    private static Command intakeSpike3(AutoBuildState state) {
        PathChain path = intakeSpike3Path(state);
        return intakeSpikeFollowingPath(state, path);
    }

    public static Command intakeWall(AutoBuildState state, boolean reverseIntake, ShootPathFlag... flags) {
        EnumSet<ShootPathFlag> flagArray = ArrayUtil.toEnumSet(flags, ShootPathFlag.class);
        state.lastPath = PathUtil.addPathBuilderCurve(state.robot, state.startPoseBlue, state.lastPath, INTAKE_WALL_CONTROL_NEAR_POSE, INTAKE_WALL_POSE_2, state.mirror, true, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .setNoDeceleration()
                .setTranslationalConstraint(WALL_TRANSLATIONAL_CONSTRAINT)
                .build();

        return new SequentialCommandGroup(
                new ParallelRaceGroup(
                        new FollowPathAndWaitForWallCommand(state.robot, state.lastPath, true, 1.0, 12.0),
                        new WaitForIntakeCommand(state.robot)
                ),
                new ConditionalCommand(
                        new InstantCommand(()->{}),
                        new WaitForIntakeCommand(state.robot).withTimeout(WALL_INTAKE_DELAY),
                        () -> flagArray.contains(FIRST_WALL_SORTED)),

                new ConditionalCommand(
                        new SetIntakeSpeedCommand(state.robot.intake, org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem.REVERSE_SPEED),
                        new InstantCommand(() -> {
                        }),
                        () -> reverseIntake && !ArrayUtil.contains(state.robot.spindexer.getBallPositions(), BallColor.NONE)
                )
        );
    }

    public static Command shootPreload(AutoBuildState state, ShootPathFlag... flagArr) {
        return UnsortedShootRoutines.shootPreload(state, flagArr);
    }

    public static Command intakeSpike(AutoBuildState state, int spikeNumber, ShootPathFlag... flagArr) {
        EnumSet<ShootPathFlag> flags = ArrayUtil.toEnumSet(flagArr, ShootPathFlag.class);
        switch (spikeNumber) {
            case 1:
                return intakeSpike1(state, flags);
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
//
//        PathChain intakeBeforeHorizPath = intakeBeforeHorizPathBuilder
//                .setConstraintsForLast(RELAXED_CONSTRAINTS)
//                .build();
//
//        state.lastPath = PathUtil.addPathBuilderLine(
//                state.robot, state.startPoseBlue,
//                        intakeBeforeHorizPath, intakePose,
//                        state.mirror, false, false)
//                .setConstantHeadingInterpolation(intakeBeforeHorizPose.mirror(state.mirror).heading)
//                .setConstraintsForLast(RELAXED_CONSTRAINTS)
//                .build();

        state.lastPath = intakeBeforeHorizPathBuilder
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .addPath(new BezierLine(
                        intakeBeforeHorizPose.mirror(state.mirror).toPedro(),
                        intakePose.mirror(state.mirror).toPedro()
                ))
                .setConstantHeadingInterpolation(intakeBeforeHorizPose.mirror(state.mirror).heading)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();

//        state.lastPath = PathUtil.addPathBuilderLine(state.robot, state.startPoseBlue, state.lastPath, intakePose, state.mirror, false, false)
//                .setConstraintsForLast(RELAXED_CONSTRAINTS)
//                .build();

        if (spikeNumber == 1) {
            return new SequentialCommandGroup(
                    new FollowPathAndWaitForWallCommand(state.robot, state.lastPath, true, MAX_DRIVETRAIN_POWER_INTAKING, 20)
                            .wallDistanceIsForRemaining(),
                    new WaitForIntakeCommand(state.robot).withTimeout(INTAKE_TIMEOUT_HORIZ)
            );
        } else {
            return new SequentialCommandGroup(
                    new FollowPathCommand(state.robot.follower, state.lastPath, true, MAX_DRIVETRAIN_POWER_INTAKING),
                    new WaitForIntakeCommand(state.robot).withTimeout(INTAKE_TIMEOUT_HORIZ)
            );
        }
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

    public static Command intakeGate(AutoBuildState state, EnumSet<ShootPathFlag> flags) {
        return new SequentialCommandGroup(
                new FollowPathCommand(state.robot.follower, intakeGatePath1(state), true, MAX_DRIVETRAIN_POWER_INTAKING),
                new ConditionalCommand(new WaitCommand(200), new InstantCommand(()->{}), () -> flags.contains(LONG_GATE_PAUSE) ),
                new FollowPathCommand(state.robot.follower, intakeGatePath2(state), true, MAX_DRIVETRAIN_POWER_INTAKING),
                new WaitForIntakeCommand(state.robot).withTimeout(flags.contains(LONG_GATE_PAUSE) ? (GATE_INTAKE_DELAY + 500) : GATE_INTAKE_DELAY)
        );
    }

    public static Command intakeGateNormal(AutoBuildState state, EnumSet<ShootPathFlag> flags) {
        state.lastPath = PathUtil.addPathBuilderCurve(state.robot, state.startPoseBlue, state.lastPath, GATE_CONTROL_POSE_NORMAL, HITTING_GATE_NORMAL, state.mirror, false, false)
                .setNoDeceleration()
                .build();
        return new SequentialCommandGroup(
                new FollowPathCommand(state.robot.follower, state.lastPath, true, MAX_DRIVETRAIN_POWER_INTAKING),
                new WaitForIntakeCommand(state.robot).withTimeout(GATE_INTAKE_DELAY)
        );
    }

    public static Command shootGate(AutoBuildState state, boolean reverseIntake, ShootPathFlag... flagArr) {
        return UnsortedShootRoutines.shootGate(state, reverseIntake, flagArr);
    }

    public static Command cycleGate(AutoBuildState state, boolean reverseIntake, ShootPathFlag... flags) {
        EnumSet<ShootPathFlag> flagArray = ArrayUtil.toEnumSet(flags, ShootPathFlag.class);
        return new SequentialCommandGroup(
                intakeGate(state, flagArray),
                shootGate(state, reverseIntake, flags)
        );
    }

    public static Command cycleGateNormal(AutoBuildState state, boolean reverseIntake, ShootPathFlag... flags) {
        EnumSet<ShootPathFlag> flagArray = ArrayUtil.toEnumSet(flags, ShootPathFlag.class);
        return new SequentialCommandGroup(
                intakeGateNormal(state, flagArray),
                shootGate(state, reverseIntake, flags)
        );
    }
}

