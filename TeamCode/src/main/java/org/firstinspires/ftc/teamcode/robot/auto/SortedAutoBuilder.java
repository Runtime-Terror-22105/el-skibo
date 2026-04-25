package org.firstinspires.ftc.teamcode.robot.auto;

import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.CAMERA_WAIT_MINIMUM_TIME;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.EARLY_SHOOT_DISTANCE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.FAR_BALL_CV_DETECTION_TIMEOUT;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.PRELOAD_PRE_SHOOT_DELAY;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.RAMP_CV_TIMEOUT;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_DELAY;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_LAST_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_SORTED_POSE_1;

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
import org.firstinspires.ftc.teamcode.robot.command.intake.SetIntakeUpCommand;
import org.firstinspires.ftc.teamcode.robot.command.shooter.ShootThreeBallsCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.PrepareShootCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.WaitForSpindexerYawCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToIntakeStateCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToRestingStateCommand;
import org.firstinspires.ftc.teamcode.robot.command.vision.WaitForGlyphCommand;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;
import org.firstinspires.ftc.teamcode.robot.subsystems.vision.CameraSubsystem;
import org.firstinspires.ftc.teamcode.util.ArrayUtil;
import org.firstinspires.ftc.teamcode.util.BallColor;

import java.util.EnumSet;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Supplier;

public final class SortedAutoBuilder {
    public static boolean TWO_SEGMENT_PARK_SORTED = false;

    private SortedAutoBuilder() {
    }

    public static Command prepareRampCV(AutoBuildState state) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> state.robot.camera.setCVMode(CameraSubsystem.FRONT_CV_MODE.RAMP)),
                new LogCatCommand("AutoBuilder", "rampcv turned on"),
                // note: intake needs to be down because otherwise it blocks the camera
                new SetIntakeUpCommand(state.robot.intake, false),
                new WaitCommand(CAMERA_WAIT_MINIMUM_TIME),
                new WaitUntilCommand(() -> state.robot.camera.getBallCountChanged()).withTimeout(RAMP_CV_TIMEOUT),
                new LogCatCommand("AutoBuilder", "ball count checker done")
        );
    }

    private static PathChain shootPreloadPath(AutoBuildState state, EnumSet<ShootPathFlag> flags) {
        Pose2d shootPose = flags.contains(ShootPathFlag.LAST) && !TWO_SEGMENT_PARK_SORTED
                ? SHOOT_LAST_POSE
                : SHOOT_SORTED_POSE_1;
        PathBuilder builder = PathUtil.addPathBuilderLine(state.robot, state.startPoseBlue, state.lastPath, shootPose, state.mirror, false, false);
        state.lastPath = builder.build();
        return state.lastPath;
    }

    private static PathChain shootSpikePath(AutoBuildState state, int spikeNumber, EnumSet<ShootPathFlag> flags) {
        Pose2d shootPose;
        boolean useTangential = flags.contains(ShootPathFlag.LAST);
        if (flags.contains(ShootPathFlag.LAST) && !TWO_SEGMENT_PARK_SORTED){
            shootPose = SHOOT_LAST_POSE;
        }
        else if (spikeNumber == 1) {
            shootPose = SHOOT_SORTED_POSE_1;
        } else if (spikeNumber == 2) {
            shootPose = AutoConstants.SHOOT_SORTED_POSE_2;
        } else if (spikeNumber == 3) {
            shootPose = AutoConstants.SHOOT_SORTED_POSE_3;
            useTangential = true;
        } else {
            throw new IllegalArgumentException("Invalid spike number: " + spikeNumber);
        }


        PathBuilder builder = PathUtil.addPathBuilderLine(state.robot, state.startPoseBlue, state.lastPath, shootPose, state.mirror, useTangential, useTangential);
        state.lastPath = builder.build();
        return state.lastPath;
    }

    static Command shootCommand(AutoBuildState state, EnumSet<ShootPathFlag> flags) {
        Command command = new ShootThreeBallsCommand(state.robot)
                .andThen(new SequentialCommandGroup(
                        new WaitForSpindexerYawCommand(state.robot.spindexer).withTimeout(500),
                        new WaitCommand(SHOOT_DELAY)
                ));

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
        shootPath.lastPath().setBrakingStrength(state.shootBrakingStrength);

        Supplier<Boolean> hasStartedPrepareShoot = () -> state.robot.robotState.equals(RobotState.READY_TO_SHOOT) || state.robot.robotState.equals(RobotState.TRANSFER);
        Supplier<Command> maybePrepareShootCommand = () -> new ConditionalCommand(
                new PrepareShootCommand(state.robot, false),
                new InstantCommand(() -> {
                }),
                () -> !(state.robot.robotState.equals(RobotState.READY_TO_SHOOT) || state.robot.robotState.equals(RobotState.TRANSFER))
        );

        return new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new FollowPathCommand(state.robot.follower, shootPath, true),
                        new InstantCommand(() -> hasFinishedPath.set(true))
                ).andThen(new LogCatCommand("AutoBuilder", "follow path done, waiting for prepare shoot")),

                new SequentialCommandGroup(
                        new WaitUntilCommand(() ->
                                hasStartedPrepareShoot.get()
                                        || hasFinishedPath.get()
                                        || !ArrayUtil.contains(state.robot.spindexer.getBallPositions(), BallColor.NONE)
                        ).withTimeout(prepareShootDelay),
                        maybePrepareShootCommand.get()
                ).andThen(new LogCatCommand("AutoBuilder", "prepare shoot done, waiting for path or distance")),

                new SequentialCommandGroup(
                        new WaitUntilCommand(() -> state.robot.robotState.equals(RobotState.READY_TO_SHOOT)),
                        new WaitUntilCommand(() -> hasFinishedPath.get() || state.robot.follower.getDistanceRemaining() < distanceConstraint),
                        new WaitCommand(AutoConstants.SORTED_SHOOT_DELAY),
                        shootCommand(state, flags)
                ).andThen(new LogCatCommand("AutoBuilder", "shoot command done"))
        ).andThen(new LogCatCommand("AutoBuilder", "createFollow end"));
    }

    public static Command shootPreload(AutoBuildState state, ShootPathFlag... flagArr) {
        EnumSet<ShootPathFlag> flags = ArrayUtil.toEnumSet(flagArr, ShootPathFlag.class);
        PathChain path = shootPreloadPath(state, flags);
        path.lastPath().setBrakingStrength(state.shootBrakingStrength);
        return new SequentialCommandGroup(
                new FollowPathCommand(state.robot.follower, path, true).alongWith(
                        new WaitCommand(500).andThen(new InstantCommand(() -> state.robot.camera.setAprilTagsEnabled(true)))
                ),
                new WaitCommand(PRELOAD_PRE_SHOOT_DELAY),
                new WaitForGlyphCommand(state.robot.camera).withTimeout(AutoConstants.WAIT_TIMEOUT_MOTIF),
                new InstantCommand(() -> state.robot.camera.setAprilTagsEnabled(false)),
                new PrepareShootCommand(state.robot, false),
                shootCommand(state, flags)
        );
    }

    public static Command shootSpike(AutoBuildState state, int spikeNumber, ShootPathFlag... flagArr) {
        EnumSet<ShootPathFlag> flags = ArrayUtil.toEnumSet(flagArr, ShootPathFlag.class);
        PathChain path = flags.contains(ShootPathFlag.PRELOAD_SHOOT_SPOT)
                ? shootPreloadPath(state, flags)
                : shootSpikePath(state, spikeNumber, flags);

        Command endCommand = new InstantCommand(() -> {
        });
        if (flags.contains(ShootPathFlag.LAST) && TWO_SEGMENT_PARK_SORTED) {
            endCommand = park(state);
        }

        return new SequentialCommandGroup(
                createFollowShootPathAndShootCommand(state, state.waitBeforeShooting, path, flags),
                endCommand
        );
    }

    public static Command park(AutoBuildState state) {
        state.lastPath = PathUtil.addPathBuilderLine(state.robot, state.startPoseBlue, state.lastPath, SHOOT_LAST_POSE, state.mirror, false, false)
                .build();
        return new FollowPathCommand(state.robot.follower, state.lastPath, true);
    }
}

