package org.firstinspires.ftc.teamcode.robot.auto;

import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.EARLY_SHOOT_DISTANCE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.FLLYWHEEL_SPIN_UP_TIMEOUT_MS;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.RELAXED_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.REVERSE_INTAKE_GATE_DELAY;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_EDGE_HORIZ_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_EDGE_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_LAST_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_PRELOAD_HORIZ_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_PRELOAD_POSE;

import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.LogCatCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ScheduleCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.command.intake.SetIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.robot.command.shooter.ShootThreeBallsCommand;
import org.firstinspires.ftc.teamcode.robot.command.shooter.WaitForFlywheelCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.PrepareShootCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToIntakeStateCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToRestingStateCommand;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;
import org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.ArrayUtil;
import org.firstinspires.ftc.teamcode.util.BallColor;

import java.util.EnumSet;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Supplier;

public final class UnsortedShootRoutines {
    private UnsortedShootRoutines() {
    }

    private enum ShootPathType {
        PRELOAD(() -> SHOOT_PRELOAD_POSE, () -> SHOOT_PRELOAD_HORIZ_POSE),
        EDGE(() -> SHOOT_EDGE_POSE, () -> SHOOT_EDGE_HORIZ_POSE);

        public final Supplier<Pose2d> normal;
        public final Supplier<Pose2d> horiz;

        ShootPathType(Supplier<Pose2d> normal, Supplier<Pose2d> horiz) {
            this.normal = normal;
            this.horiz = horiz;
        }
    }

    private static Pose2d getShootPose(ShootPathType type, EnumSet<ShootPathFlag> flags) {
        if (flags.contains(ShootPathFlag.LAST)) {
            return SHOOT_LAST_POSE;
        }
        boolean isHoriz = flags.contains(ShootPathFlag.NEXT_HORIZ);
        return isHoriz ? type.horiz.get() : type.normal.get();
    }

    private static PathChain shootPreloadPath(AutoBuildState state, EnumSet<ShootPathFlag> flags) {
        PathBuilder builder = PathUtil.addPathBuilderLine(state.robot, state.startPoseBlue, state.lastPath, getShootPose(ShootPathType.PRELOAD, flags), state.mirror, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS);
        state.lastPath = builder.build();
        return state.lastPath;
    }

    private static PathChain shootSpikePath(AutoBuildState state, EnumSet<ShootPathFlag> flags) {
        PathBuilder builder = PathUtil.addPathBuilderLine(state.robot, state.startPoseBlue, state.lastPath, getShootPose(ShootPathType.EDGE, flags), state.mirror, true, true)
                .setConstraintsForLast(RELAXED_CONSTRAINTS);
        state.lastPath = builder.build();
        return state.lastPath;
    }

    private static PathChain shootGatePath(AutoBuildState state, EnumSet<ShootPathFlag> flags) {
        state.lastPath = PathUtil.addPathBuilderLine(state.robot, state.startPoseBlue, state.lastPath, getShootPose(ShootPathType.EDGE, flags), state.mirror, true, true)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
        return state.lastPath;
    }

    static Command shootCommand(AutoBuildState state, EnumSet<ShootPathFlag> flags) {
        Command command = new ShootThreeBallsCommand(state.robot);

        if (!flags.contains(ShootPathFlag.LAST)) {
            command = command.andThen(
                    new GoToRestingStateCommand(state.robot),
                    new WaitUntilCommand(() -> state.robot.spindexer.isWallDown()),
                    new GoToIntakeStateCommand(state.robot),
                    new LogCatCommand("AutoBuilder", "ending shoot")
            );
        }

        if (flags.contains(ShootPathFlag.EARLY_LEAVE)) {
            command = new SequentialCommandGroup(
                    new ScheduleCommand(command),
                    new WaitCommand(600)
            );
        }
        return command.andThen(new IncrementNumCyclesCommand(state.auto));
    }

    static Command createFollowShootPathAndShootCommand(AutoBuildState state, PathChain shootPath, EnumSet<ShootPathFlag> flags) {
        AtomicBoolean hasFinishedPath = new AtomicBoolean(false);
        double distanceConstraint = flags.contains(ShootPathFlag.EARLY_SHOOT) ? EARLY_SHOOT_DISTANCE : 0.0;
        shootPath.lastPath().setBrakingStrength(state.shootBrakingStrength);

        Supplier<Command> maybePrepareShootCommand = () -> new ConditionalCommand(
                new PrepareShootCommand(state.robot, false),
                new InstantCommand(() -> {}),
                () -> !(state.robot.robotState.equals(RobotState.READY_TO_SHOOT) || state.robot.robotState.equals(RobotState.TRANSFER))
        );

        return new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new FollowPathCommand(state.robot.follower, shootPath, false),
                        new InstantCommand(() -> hasFinishedPath.set(true))
                ).andThen(new LogCatCommand("AutoBuilder", "follow path done, waiting for prepare shoot")),

                new SequentialCommandGroup(maybePrepareShootCommand.get())
                        .andThen(new LogCatCommand("AutoBuilder", "prepare shoot done, waiting for path or distance")),

                new SequentialCommandGroup(
                        new WaitUntilCommand(() -> state.robot.robotState.equals(RobotState.READY_TO_SHOOT)),
                        new WaitUntilCommand(() -> hasFinishedPath.get() || state.robot.follower.getDistanceRemaining() < distanceConstraint),
                        new WaitCommand(AutoConstants.NORMAL_SHOOT_DELAY),
                        shootCommand(state, flags)
                ).andThen(new LogCatCommand("AutoBuilder", "shoot command done"))
        ).andThen(new LogCatCommand("AutoBuilder", "createFollow end"));
    }

    public static Command shootPreload(AutoBuildState state, ShootPathFlag... flagArr) {
        EnumSet<ShootPathFlag> flags = ArrayUtil.toEnumSet(flagArr, ShootPathFlag.class);
        PathChain path = shootPreloadPath(state, flags);
        path.lastPath().setBrakingStrength(state.shootBrakingStrength);

        if (flags.contains(ShootPathFlag.SOTM)) {
            return new ParallelCommandGroup(
                    new SequentialCommandGroup(
                            new WaitForFlywheelCommand(state.robot.shooter).withTimeout(FLLYWHEEL_SPIN_UP_TIMEOUT_MS),
                            new WaitCommand(250),
                            shootCommand(state, flags)
                    ),
                    new FollowPathCommand(state.robot.follower, path, false)
            );
        }

        return new SequentialCommandGroup(
                new FollowPathCommand(state.robot.follower, path, false),
//                new WaitCommand(PRELOAD_PRE_SHOOT_DELAY),
                new WaitForFlywheelCommand(state.robot.shooter).withTimeout(FLLYWHEEL_SPIN_UP_TIMEOUT_MS),
                shootCommand(state, flags)
        );
    }

    public static Command shootSpike(AutoBuildState state, int spikeNumber, ShootPathFlag... flagArr) {
        EnumSet<ShootPathFlag> flags = ArrayUtil.toEnumSet(flagArr, ShootPathFlag.class);
        PathChain path = flags.contains(ShootPathFlag.PRELOAD_SHOOT_SPOT)
                ? shootPreloadPath(state, flags)
                : shootSpikePath(state, flags);

        return createFollowShootPathAndShootCommand(state, path, flags);
    }

    public static Command shootGate(AutoBuildState state, boolean reverseIntake, ShootPathFlag... flagArr) {
        EnumSet<ShootPathFlag> flags = ArrayUtil.toEnumSet(flagArr, ShootPathFlag.class);
        return new ParallelCommandGroup(
                createFollowShootPathAndShootCommand(state, shootGatePath(state, flags), flags),
                new SequentialCommandGroup(
                        new WaitCommand(REVERSE_INTAKE_GATE_DELAY),
                        new ConditionalCommand(
                                new SetIntakeSpeedCommand(state.robot.intake, IntakeSubsystem.REVERSE_SPEED),
                                new InstantCommand(() -> {}),
                                () -> reverseIntake && !ArrayUtil.contains(state.robot.spindexer.getBallPositions(), BallColor.NONE)
                        )
                )
        );
    }

    public static Command shootWall(AutoBuildState state, ShootPathFlag... flagArr) {
        EnumSet<ShootPathFlag> flags = ArrayUtil.toEnumSet(flagArr, ShootPathFlag.class);
        if (flags.contains(ShootPathFlag.FIRST_WALL_SORTED)){
            state.lastPath = PathUtil.addPathBuilderCurve(state.robot, state.startPoseBlue, state.lastPath, AutoConstants.INTAKE_WALL_CONTROL_POSE, AutoConstants.SHOOT_FAR_POSE, state.mirror, false, false)
                    .setConstraintsForLast(RELAXED_CONSTRAINTS)
                    .build();
        }
        else {
            Pose2d pose;
            if (flags.contains(ShootPathFlag.FORWARD_FACING_SHOOT_SPOT)) pose = AutoConstants.SHOOT_FAR_POSE_FORWARD_FACING;
            else pose = AutoConstants.SHOOT_FAR_POSE;
            state.lastPath = PathUtil.addPathBuilderLine(state.robot, state.startPoseBlue, state.lastPath, pose, state.mirror, false, false)
                    .setConstraintsForLast(RELAXED_CONSTRAINTS)
                    .build();
        }

        return createFollowShootPathAndShootCommand(state, state.lastPath, flags);
    }
}

