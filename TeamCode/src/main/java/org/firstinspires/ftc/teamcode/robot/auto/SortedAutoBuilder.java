package org.firstinspires.ftc.teamcode.robot.auto;

import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.EARLY_SHOOT_DISTANCE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.PRELOAD_PRE_SHOOT_DELAY;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.RELAXED_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.REVERSE_INTAKE_GATE_DELAY;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_DELAY;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_EDGE_HORIZ_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_EDGE_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_LAST_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_PRELOAD_HORIZ_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_PRELOAD_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_SORTED_POSE;

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
import org.firstinspires.ftc.teamcode.robot.command.spindexer.WaitForSpindexerYawCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToIntakeStateCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToRestingStateCommand;
import org.firstinspires.ftc.teamcode.robot.command.vision.WaitForGlyphCommand;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;
import org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.ArrayUtil;
import org.firstinspires.ftc.teamcode.util.BallColor;

import java.util.EnumSet;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Supplier;

public final class SortedAutoBuilder {
    public static boolean TWO_SEGMENT_PARK_SORTED = false;

    private SortedAutoBuilder() {
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

    private static Pose2d getShootPose(AutoBuildState state, ShootPathType type, EnumSet<ShootPathFlag> flags) {
        if (flags.contains(ShootPathFlag.LAST) && (!state.auto.wantsAutoSort() || !TWO_SEGMENT_PARK_SORTED)) {
            return SHOOT_LAST_POSE;
        }

        if (state.auto.wantsAutoSort()) {
            return SHOOT_SORTED_POSE;
        }

        boolean isHoriz = flags.contains(ShootPathFlag.NEXT_HORIZ);
        return isHoriz ? type.horiz.get() : type.normal.get();
    }

    static PathChain shootPreloadPath(AutoBuildState state, EnumSet<ShootPathFlag> flags) {
        PathBuilder builder = PathUtil.addPathBuilderLine(state.robot, state.startPoseBlue, state.lastPath, getShootPose(state, ShootPathType.PRELOAD, flags), state.mirror, false, false);
        if (!state.auto.wantsAutoSort()) {
            builder = builder.setConstraintsForLast(RELAXED_CONSTRAINTS);
        }
        state.lastPath = builder.build();
        return state.lastPath;
    }

    private static PathChain shootSpikePath(AutoBuildState state, EnumSet<ShootPathFlag> flags) {
        boolean useTangential = !state.auto.wantsAutoSort() || flags.contains(ShootPathFlag.LAST);
        PathBuilder builder = PathUtil.addPathBuilderLine(state.robot, state.startPoseBlue, state.lastPath, getShootPose(state, ShootPathType.EDGE, flags), state.mirror, useTangential, useTangential);
        if (!state.auto.wantsAutoSort()) {
            builder = builder.setConstraintsForLast(RELAXED_CONSTRAINTS);
        }
        state.lastPath = builder.build();
        return state.lastPath;
    }

    private static PathChain shootGatePath(AutoBuildState state, EnumSet<ShootPathFlag> flags) {
        state.lastPath = PathUtil.addPathBuilderLine(state.robot, state.startPoseBlue, state.lastPath, getShootPose(state, ShootPathType.EDGE, flags), state.mirror, true, true)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
        return state.lastPath;
    }

    static Command shootCommand(AutoBuildState state, EnumSet<ShootPathFlag> flags) {
        Command command = new ShootThreeBallsCommand(state.robot);
        if (state.auto.wantsAutoSort()) {
            command = command.andThen(new SequentialCommandGroup(
                    new WaitForSpindexerYawCommand(state.robot.spindexer).withTimeout(500),
                    new WaitCommand(SHOOT_DELAY)
            ));
        }

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

    static Command createFollowShootPathAndShootCommand(AutoBuildState state, long prepareShootDelay, PathChain shootPath, EnumSet<ShootPathFlag> flags) {
        AtomicBoolean hasFinishedPath = new AtomicBoolean(false);
        double distanceConstraint = flags.contains(ShootPathFlag.EARLY_SHOOT) ? EARLY_SHOOT_DISTANCE : 0.0;
        long shootDelay = state.auto.wantsAutoSort() ? AutoConstants.SORTED_SHOOT_DELAY : AutoConstants.NORMAL_SHOOT_DELAY;
        boolean holdEnd = state.auto.wantsAutoSort();
        shootPath.lastPath().setBrakingStrength(state.shootBrakingStrength);

        Supplier<Boolean> hasStartedPrepareShoot = () -> state.robot.robotState.equals(RobotState.READY_TO_SHOOT) || state.robot.robotState.equals(RobotState.TRANSFER);
        Supplier<Command> maybePrepareShootCommand = () -> new ConditionalCommand(
                new PrepareShootCommand(state.robot, state.prepareShootTimeBeforeReverseIntake, true),
                new InstantCommand(() -> {
                }),
                () -> !(state.robot.robotState.equals(RobotState.READY_TO_SHOOT) || state.robot.robotState.equals(RobotState.TRANSFER))
        );

        return new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new FollowPathCommand(state.robot.follower, shootPath, holdEnd),
                        new InstantCommand(() -> hasFinishedPath.set(true))
                ).andThen(new LogCatCommand("AutoBuilder", "follow path done, waiting for prepare shoot")),

                new SequentialCommandGroup(
                        new WaitUntilCommand(() ->
                                !state.auto.wantsAutoSort()
                                        || hasStartedPrepareShoot.get()
                                        || hasFinishedPath.get()
                                        || !ArrayUtil.contains(state.robot.spindexer.getBallPositions(), BallColor.NONE)
                        ).withTimeout(prepareShootDelay),
                        maybePrepareShootCommand.get()
                ).andThen(new LogCatCommand("AutoBuilder", "prepare shoot done, waiting for path or distance")),

                new SequentialCommandGroup(
                        new WaitUntilCommand(() -> state.robot.robotState.equals(RobotState.READY_TO_SHOOT)),
                        new WaitUntilCommand(() ->
                                hasFinishedPath.get() || state.robot.follower.getDistanceRemaining() < distanceConstraint
                        ),
                        new WaitCommand(shootDelay),
                        shootCommand(state, flags)
                ).andThen(new LogCatCommand("AutoBuilder", "shoot command done"))
        ).andThen(new LogCatCommand("AutoBuilder", "createFollow end"));
    }

    public static Command shootPreload(AutoBuildState state, ShootPathFlag... flagArr) {
        boolean wantsAutoSort = state.robot.getAutoSort();
        EnumSet<ShootPathFlag> flags = ArrayUtil.toEnumSet(flagArr, ShootPathFlag.class);
        PathChain path = shootPreloadPath(state, flags);
        path.lastPath().setBrakingStrength(state.shootBrakingStrength);
        if (wantsAutoSort) {
            return new SequentialCommandGroup(
                    new FollowPathCommand(state.robot.follower, path, true).alongWith(
                            new WaitCommand(500).andThen(new InstantCommand(() -> state.robot.camera.setAprilTagsEnabled(true)))
                    ),
                    new WaitCommand(PRELOAD_PRE_SHOOT_DELAY),
                    new WaitForGlyphCommand(state.robot.camera).withTimeout(AutoConstants.WAIT_TIMEOUT_MOTIF),
                    new InstantCommand(() -> state.robot.camera.setAprilTagsEnabled(false)),
                    new PrepareShootCommand(state.robot),
                    shootCommand(state, flags)
            );
        }

        if (flags.contains(ShootPathFlag.SOTM)) {
            return new ParallelCommandGroup(
                    new SequentialCommandGroup(
                            new WaitForFlywheelCommand(state.robot.shooter).withTimeout(625),
                            new WaitCommand(250),
                            shootCommand(state, flags)
                    ),
                    new FollowPathCommand(state.robot.follower, path, false)
            );
        }

        return new SequentialCommandGroup(
                new FollowPathCommand(state.robot.follower, path, false),
                new WaitCommand(PRELOAD_PRE_SHOOT_DELAY),
                shootCommand(state, flags)
        );
    }

    public static Command shootSpike(AutoBuildState state, int spikeNumber, ShootPathFlag... flagArr) {
        EnumSet<ShootPathFlag> flags = ArrayUtil.toEnumSet(flagArr, ShootPathFlag.class);
        PathChain path = flags.contains(ShootPathFlag.PRELOAD_SHOOT_SPOT)
                ? shootPreloadPath(state, flags)
                : shootSpikePath(state, flags);

        Command endCommand = new InstantCommand(() -> {
        });
        if (flags.contains(ShootPathFlag.LAST) && state.auto.wantsAutoSort() && TWO_SEGMENT_PARK_SORTED) {
            endCommand = parkSorted(state);
        }

        return new SequentialCommandGroup(
                createFollowShootPathAndShootCommand(state, state.waitBeforeShooting, path, flags),
                endCommand
        );
    }

    public static Command shootGate(AutoBuildState state, boolean reverseIntake, ShootPathFlag... flagArr) {
        EnumSet<ShootPathFlag> flags = ArrayUtil.toEnumSet(flagArr, ShootPathFlag.class);
        return new ParallelCommandGroup(
                createFollowShootPathAndShootCommand(state, 250, shootGatePath(state, flags), flags),
                new SequentialCommandGroup(
                        new WaitCommand(REVERSE_INTAKE_GATE_DELAY),
                        new ConditionalCommand(
                                new SetIntakeSpeedCommand(state.robot.intake, IntakeSubsystem.REVERSE_SPEED),
                                new InstantCommand(() -> {
                                }),
                                () -> reverseIntake && !ArrayUtil.contains(state.robot.spindexer.getBallPositions(), BallColor.NONE)
                        )
                )
        );
    }

    public static Command shootWall(AutoBuildState state, ShootPathFlag... flagArr) {
        EnumSet<ShootPathFlag> flags = ArrayUtil.toEnumSet(flagArr, ShootPathFlag.class);
        state.lastPath = PathUtil.addPathBuilderLine(state.robot, state.startPoseBlue, state.lastPath, AutoConstants.SHOOT_FAR_POSE, state.mirror, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
        return createFollowShootPathAndShootCommand(state, 250, state.lastPath, flags);
    }

    public static Command parkSorted(AutoBuildState state) {
        state.lastPath = PathUtil.addPathBuilderLine(state.robot, state.startPoseBlue, state.lastPath, SHOOT_LAST_POSE, state.mirror, false, false)
                .build();
        return new FollowPathCommand(state.robot.follower, state.lastPath, true);
    }
}

