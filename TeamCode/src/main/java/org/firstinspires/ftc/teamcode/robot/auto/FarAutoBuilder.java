package org.firstinspires.ftc.teamcode.robot.auto;

import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.CAMERA_WAIT_MINIMUM_TIME;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.CONTROL_POSE_LONG_INTAKE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.END_POSE_LONG_INTAKE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.FAR_BALL_CV_DETECTION_TIMEOUT;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.GATE_INTAKE_DELAY;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_2_CONTROL_FAR;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_2_POSE_FAR;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_3_POSE_FAR;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_DELAY;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_TUNNEL_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_WALL_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_WALL_TIMEOUT_DISTANCE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_WALL_VISION_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.MAX_DRIVETRAIN_POWER_INTAKING;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.PRELOAD_FAR_PRE_SHOOT_SPINUP_TIMEOUT;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.PREPARE_INTAKE_2_CONTROL_FAR;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.PREPARE_INTAKE_2_POSE_FAR;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.PREPARE_INTAKE_3_CONTROL_FAR;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.PREPARE_INTAKE_3_POSE_FAR;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.RELAXED_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_FAR_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.START_POSE_LONG_INTAKE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.WALL_INTAKE_DELAY;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.WALL_TRANSLATIONAL_CONSTRAINT;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.HeadingInterpolator;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.DeferredCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.LogCatCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.pedroPathing.FixedHeadingInterpolator;
import org.firstinspires.ftc.teamcode.robot.command.WaitForIntakeCommand;
import org.firstinspires.ftc.teamcode.robot.command.intake.SetIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.robot.command.intake.SetIntakeUpCommand;
import org.firstinspires.ftc.teamcode.robot.command.shooter.WaitForFlywheelCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToIntakeStateCommand;
import org.firstinspires.ftc.teamcode.robot.command.vision.StopScanningForGlyphsCommand;
import org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.ArrayUtil;
import org.firstinspires.ftc.teamcode.util.BallColor;

import java.util.EnumSet;

public final class FarAutoBuilder {
    private FarAutoBuilder() {
    }

    public static Command shootPreload(AutoBuildState state, ShootPathFlag... flagArr) {
        EnumSet<ShootPathFlag> flags = ArrayUtil.toEnumSet(flagArr, ShootPathFlag.class);
//        state.lastPath = PathUtil.addPathBuilderLine(state.robot, state.startPoseBlue, state.lastPath, SHOOT_PRELOAD_FAR_POSE, state.mirror, false, false)
//                .build();
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
//                        new FollowPathCommand(state.robot.follower, state.lastPath, true),
                        new SequentialCommandGroup(
                                new WaitForFlywheelCommand(state.robot.shooter).withTimeout(PRELOAD_FAR_PRE_SHOOT_SPINUP_TIMEOUT),
                                new WaitCommand(250)
                        )
                ),
                UnsortedShootRoutines.shootCommand(state, flags),
                new StopScanningForGlyphsCommand(state.robot.camera)
        );
    }

    public static Command intakeSpike3(AutoBuildState state) {
        state.lastPath = PathUtil.addPathBuilderCurve(state.robot, state.startPoseBlue, state.lastPath, PREPARE_INTAKE_3_CONTROL_FAR, PREPARE_INTAKE_3_POSE_FAR, state.mirror, false, false)
                .addPath(
                        new BezierLine(PREPARE_INTAKE_3_POSE_FAR.toPedro(state.mirror), INTAKE_3_POSE_FAR.toPedro(state.mirror))
                )
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
        return new SequentialCommandGroup(
                new ParallelRaceGroup(
                        new FollowPathAndWaitForWallCommand(state.robot, state.lastPath, true, MAX_DRIVETRAIN_POWER_INTAKING, 24.0),
                        new WaitForIntakeCommand(state.robot)
                )
//                new WaitForIntakeCommand(state.robot).withTimeout(INTAKE_DELAY)
        );
    }

    public static Command intakeSpike2AndPushGate(AutoBuildState state) {
        state.lastPath = PathUtil.addPathBuilderCurve(state.robot, state.startPoseBlue, state.lastPath, PREPARE_INTAKE_2_CONTROL_FAR, PREPARE_INTAKE_2_POSE_FAR, state.mirror, true, false)
//                .setHeadingInterpolation(
//                        HeadingInterpolator.piecewise(
//                                new HeadingInterpolator.PiecewiseNode(0, 0.7,
//                                        HeadingInterpolator.linear(
//                                                SHOOT_FAR_POSE_FORWARD_FACING.mirror(state.mirror).heading,
//                                                PREPARE_INTAKE_2_POSE_FAR.mirror(state.mirror).heading
//                                        )
//                                ),
//                                new HeadingInterpolator.PiecewiseNode(0.7, 1.0,
//                                        HeadingInterpolator.constant(PREPARE_INTAKE_2_POSE_FAR.mirror(state.mirror).heading)
//                                )
//                        )
//                )
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
        return new SequentialCommandGroup(
                new ParallelRaceGroup(
                        new SequentialCommandGroup(
                                new FollowPathAndWaitForWallCommand(state.robot, state.lastPath, true, MAX_DRIVETRAIN_POWER_INTAKING, 12.0),
                                intakeSpike2(state)
                        ),
                        new WaitForIntakeCommand(state.robot).alongWith(new WaitCommand(GATE_INTAKE_DELAY))
                ),
                new WaitForIntakeCommand(state.robot).withTimeout(INTAKE_DELAY)
        );
    }

    public static Command intakeSpike2(AutoBuildState state) {
        state.lastPath = PathUtil.addPathBuilderCurve(state.robot, state.startPoseBlue, state.lastPath, INTAKE_2_CONTROL_FAR, INTAKE_2_POSE_FAR, state.mirror, false, false)
                .setConstantHeadingInterpolation(Math.toRadians(20))
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
        return new FollowPathCommand(state.robot.follower, state.lastPath, false, MAX_DRIVETRAIN_POWER_INTAKING);
    }

    public static Command shootSpike3(AutoBuildState state, ShootPathFlag... flagArr) {
        EnumSet<ShootPathFlag> flags = ArrayUtil.toEnumSet(flagArr, ShootPathFlag.class);
        state.lastPath = PathUtil.addPathBuilderLine(state.robot, state.startPoseBlue, state.lastPath, SHOOT_FAR_POSE, state.mirror, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
        return UnsortedShootRoutines.createFollowShootPathAndShootCommand(state, state.lastPath, flags);
    }

    public static Command intakeWall(AutoBuildState state, boolean reverseIntake) {
        state.lastPath = PathUtil.addPathBuilderLine(state.robot, state.startPoseBlue, state.lastPath, INTAKE_WALL_POSE, state.mirror, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
//                .setNoDeceleration()
                .setTranslationalConstraint(WALL_TRANSLATIONAL_CONSTRAINT)
//                .curveThrough(0, INTAKE_WALL_POSE_2.toPedro(state.mirror))
//                .setConstraintsForLast(RELAXED_CONSTRAINTS)
//                .setTranslationalConstraint(WALL_TRANSLATIONAL_CONSTRAINT)
//                .setHeadingConstraint(0.12)
                .build();
        return new SequentialCommandGroup(
                new ParallelRaceGroup(
                        new FollowPathAndWaitForWallCommand(state.robot, state.lastPath, true, 1.0, INTAKE_WALL_TIMEOUT_DISTANCE),
                        new WaitForIntakeCommand(state.robot)
                ),
                new WaitForIntakeCommand(state.robot).withTimeout(WALL_INTAKE_DELAY),
                new ConditionalCommand(
                        new SetIntakeSpeedCommand(state.robot.intake, IntakeSubsystem.REVERSE_SPEED),
                        new InstantCommand(() -> {}),
                        () -> reverseIntake && !ArrayUtil.contains(state.robot.spindexer.getBallPositions(), BallColor.NONE)
                )
        );
    }

    public static Command prepareVision(AutoBuildState state) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> state.robot.camera.setBallPipelineEnabled(true)),
                new LogCatCommand("AutoBuilder", "finished path to vision, waiting for blob"),
                // note: intake needs to be down because otherwise it blocks the camera
                new SetIntakeUpCommand(state.robot.intake, false),
                new WaitCommand(CAMERA_WAIT_MINIMUM_TIME),
                new WaitUntilCommand(() -> state.robot.camera.hasBlob()).withTimeout(FAR_BALL_CV_DETECTION_TIMEOUT),
                new LogCatCommand("AutoBuilder", "blob found, preparing shoot")
        );
    }

    public static Command intakeVision(AutoBuildState state, boolean reverseIntake) {
        Pose2d wallCoords = INTAKE_WALL_VISION_POSE.mirror(state.mirror);
        state.lastPath = PathUtil.addPathBuilderLine(state.robot, state.startPoseBlue, state.lastPath, state.robot.camera.offsetByBallCoords(wallCoords), false, true, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .setNoDeceleration()
                .setTranslationalConstraint(WALL_TRANSLATIONAL_CONSTRAINT)
                .build();
        return new SequentialCommandGroup(
                new InstantCommand(() -> state.robot.camera.setBallPipelineEnabled(false)),
                new GoToIntakeStateCommand(state.robot),
                new FollowPathCommand(state.robot.follower, state.lastPath, true),
                new WaitForIntakeCommand(state.robot).withTimeout(WALL_INTAKE_DELAY),
                new ConditionalCommand(
                        new SetIntakeSpeedCommand(state.robot.intake, org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem.REVERSE_SPEED),
                        new InstantCommand(() -> {}),
                        () -> reverseIntake && !ArrayUtil.contains(state.robot.spindexer.getBallPositions(), BallColor.NONE)
                ),
                new InstantCommand(() -> {
                    state.robot.camera.resetBlob();
                    state.robot.camera.setBallPipelineEnabled(false);
                })
        );
    }

    public static Command intakeTunnel(AutoBuildState state, boolean reverseIntake) {
        state.lastPath = PathUtil.addPathBuilderLine(state.robot, state.startPoseBlue, state.lastPath, INTAKE_TUNNEL_POSE, state.mirror, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .setNoDeceleration()
                .build();
        return new SequentialCommandGroup(
                new ParallelRaceGroup(
                        new FollowPathAndWaitForWallCommand(state.robot, state.lastPath, true, 1.0, 20.0),
                        new WaitForIntakeCommand(state.robot)
                ),
                new WaitForIntakeCommand(state.robot).withTimeout(WALL_INTAKE_DELAY),
                new ConditionalCommand(
                        new SetIntakeSpeedCommand(state.robot.intake, org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem.REVERSE_SPEED),
                        new InstantCommand(() -> {}),
                        () -> reverseIntake && !ArrayUtil.contains(state.robot.spindexer.getBallPositions(), BallColor.NONE)
                )
        );
    }

    public static Command shootWall(AutoBuildState state, ShootPathFlag... flagArr) {
        return UnsortedShootRoutines.shootWall(state, flagArr);
    }

    public static Command cycleTunnel(AutoBuildState state, boolean reverseIntake, ShootPathFlag... flagArr) {
        return new SequentialCommandGroup(
                intakeTunnel(state, reverseIntake),
                shootWall(state, flagArr)
        );
    }

    public static Command cycleWall(AutoBuildState state, boolean reverseIntake, ShootPathFlag... flagArr) {
        return new SequentialCommandGroup(
                intakeWall(state, reverseIntake),
                shootWall(state, flagArr)
        );
    }

    public static Command cycleVision(AutoBuildState state, boolean reverseIntake, ShootPathFlag... flagArr) {
        return new DeferredCommand(() -> new SequentialCommandGroup(
                new LogCatCommand("AutoBuilder", "running cycle vision!!!"),
                intakeVision(state, reverseIntake),
                new ParallelCommandGroup(
                        shootWall(state, flagArr),
                        prepareVision(state))

        ), null);
    }

    public static Command intakeWallLong(AutoBuildState state) {
        state.lastPath = PathUtil.addPathBuilderLine(state.robot, state.startPoseBlue, state.lastPath, INTAKE_WALL_POSE, state.mirror, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .setNoDeceleration()
                .setTranslationalConstraint(WALL_TRANSLATIONAL_CONSTRAINT)
                .build();
        return new SequentialCommandGroup(
                new GoToIntakeStateCommand(state.robot),
                new FollowPathCommand(state.robot.follower, state.lastPath, true)
        );
    }

    public static Command controlPathLong(AutoBuildState state) {
        state.lastPath = PathUtil.addPathBuilderCurve(state.robot, state.startPoseBlue, state.lastPath, CONTROL_POSE_LONG_INTAKE, START_POSE_LONG_INTAKE, state.mirror, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .setNoDeceleration()
                .build();
        return new SequentialCommandGroup(new FollowPathCommand(state.robot.follower, state.lastPath));
    }

    public static Command intakeTunnelLong(AutoBuildState state) {
        state.lastPath = PathUtil.addPathBuilderLine(state.robot, state.startPoseBlue, state.lastPath, END_POSE_LONG_INTAKE, state.mirror, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .setNoDeceleration()
                .build();
        return new SequentialCommandGroup(new FollowPathCommand(state.robot.follower, state.lastPath));
    }

    public static Command shootWallLong(AutoBuildState state, ShootPathFlag... flagArr) {
        EnumSet<ShootPathFlag> flags = ArrayUtil.toEnumSet(flagArr, ShootPathFlag.class);
        state.lastPath = PathUtil.addPathBuilderLine(state.robot, state.startPoseBlue, state.lastPath, SHOOT_FAR_POSE, state.mirror, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(0, 0.7,
                                        FixedHeadingInterpolator.linearFromPoint(
                                                () -> state.robot.follower.getHeading(),
                                                SHOOT_FAR_POSE.heading,
                                                0, 0.5
                                        )
                                ),
                                new HeadingInterpolator.PiecewiseNode(0.7, 1.0,
                                        HeadingInterpolator.constant(SHOOT_FAR_POSE.heading)
                                )
                        )
                )
                .build();
        return UnsortedShootRoutines.createFollowShootPathAndShootCommand(state, state.lastPath, flags);
    }

    public static Command cycleLong(AutoBuildState state, boolean reverseIntake, ShootPathFlag... flagArr) {
        return new SequentialCommandGroup(
                new ParallelRaceGroup(
                        new SequentialCommandGroup(
                                intakeWallLong(state),
                                controlPathLong(state),
                                intakeTunnelLong(state)
                        ),
                        new WaitForIntakeCommand(state.robot)
                ),
                new ConditionalCommand(
                        new SetIntakeSpeedCommand(state.robot.intake, org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem.REVERSE_SPEED),
                        new InstantCommand(() -> {}),
                        () -> reverseIntake && !ArrayUtil.contains(state.robot.spindexer.getBallPositions(), BallColor.NONE)
                ),
                shootWallLong(state, flagArr)
        );
    }
}

