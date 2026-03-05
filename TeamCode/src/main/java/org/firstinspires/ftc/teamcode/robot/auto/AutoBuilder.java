package org.firstinspires.ftc.teamcode.robot.auto;

import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.AFTER_GATE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.CONTROL_POSE_LONG_INTAKE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.EARLY_SHOOT_DISTANCE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.END_POSE_LONG_INTAKE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.GATE_CONTROL_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.GATE_CONTROL_POSE_2;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.GATE_INTAKE_DELAY;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.HITTING_GATE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_1_CONTROL;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_1_HORIZ_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_1_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_2_CONTROL;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_2_HORIZ_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_2_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_3_CONTROL;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_3_CONTROL_FAR;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_3_HORIZ_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_3_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_3_POSE_FAR;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_BEFORE_HORIZ_CONTROL;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_BEFORE_HORIZ_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_DELAY;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_DELAY_HORIZ;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_TUNNEL_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_WALL_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.MAX_DRIVETRAIN_POWER_INTAKING;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.PRELOAD_FAR_PRE_SHOOT_DELAY;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.PRELOAD_PRE_SHOOT_DELAY;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.PREPARE_INTAKE_3_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.PREPARE_PUSH_GATE_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.PUSH_GATE_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.RELAXED_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.REVERSE_INTAKE_GATE_DELAY;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_BRAKING_STRENGTH;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_DELAY;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_EDGE_HORIZ_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_EDGE_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_FAR_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_LAST_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_PRELOAD_HORIZ_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_PRELOAD_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_SORTED_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.START_POSE_LONG_INTAKE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.VISION_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.WALL_INTAKE_DELAY;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.DeferredCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.LogCatCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.ScheduleCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.opmodes.auto.OneAutoToRuleThemAll;
import org.firstinspires.ftc.teamcode.pedroPathing.FixedHeadingInterpolator;
import org.firstinspires.ftc.teamcode.robot.command.WaitForIntakeCommand;
import org.firstinspires.ftc.teamcode.robot.command.intake.SetIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.robot.command.shooter.ShootThreeBallsCommand;
import org.firstinspires.ftc.teamcode.robot.command.shooter.WaitForFlywheelCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.PrepareShootCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.WaitForSpindexerYawCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToIntakeStateCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToRestingStateCommand;
import org.firstinspires.ftc.teamcode.robot.command.vision.StopScanningForGlyphsCommand;
import org.firstinspires.ftc.teamcode.robot.command.vision.WaitForGlyphCommand;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;
import org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.ArrayUtil;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.StartConfig;

import java.util.EnumSet;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Supplier;

@Config
public class AutoBuilder {
    public final Pose2d startPoseBlue;
    public final Robot robot;
    public final boolean mirror;
    public final OneAutoToRuleThemAll auto;
    private PathChain lastPath = null;

    private long waitBeforeShooting;
    public long prepareShootTimeBeforeReverseIntake = PrepareShootCommand.TIME_BEFORE_REVERSE_INTAKE;
    public double shootBrakingStrength = SHOOT_BRAKING_STRENGTH;

    public AutoBuilder(OneAutoToRuleThemAll auto, Robot robot, Team team, StartConfig initial) {
        this.auto = auto;
        this.robot = robot;
        // NB: We do not mirror the start pose here because the path builder's mirror parameter
        // will handle it for us.
        this.startPoseBlue = initial.getStartPoseBlue();
        this.mirror = Team.RED.equals(team);
        this.waitBeforeShooting = 0;
    }

    public void waitBeforeShooting(long time) {
        this.waitBeforeShooting += time;
    }

    // For NEAR ZONE
    private enum ShootPathType {
        PRELOAD(() -> SHOOT_PRELOAD_POSE, () -> SHOOT_PRELOAD_HORIZ_POSE),
        EDGE(() -> SHOOT_EDGE_POSE, () -> SHOOT_EDGE_HORIZ_POSE);

        public final Supplier<Pose2d> normal, horiz;

        ShootPathType(Supplier<Pose2d> normal, Supplier<Pose2d> horiz) {
            this.normal = normal;
            this.horiz = horiz;
        }
    }

    // Flags defined in ShootPathFlags
    private Pose2d getShootPose(ShootPathType type, EnumSet<ShootPathFlag> flags) {
        if (flags.contains(ShootPathFlag.LAST) && !auto.wantsAutoSort()) {
            return SHOOT_LAST_POSE;
        }

        if (auto.wantsAutoSort()) {
            return SHOOT_SORTED_POSE;
        }

        boolean isHoriz = flags.contains(ShootPathFlag.NEXT_HORIZ);
        return isHoriz ? type.horiz.get() : type.normal.get();
    }

    public PathChain getLastPath() {
        return lastPath;
    }

    private PathChain shootPreloadPath(EnumSet<ShootPathFlag> flags) {
        PathBuilder builder = PathUtil.addPathBuilderLine(robot, startPoseBlue, lastPath, getShootPose(ShootPathType.PRELOAD, flags), mirror, false, false);
        if (!auto.wantsAutoSort()) {
            builder = builder.setConstraintsForLast(RELAXED_CONSTRAINTS);
        }
        lastPath = builder.build();
        return lastPath;
    }

    private PathChain shootSpikePath(EnumSet<ShootPathFlag> flags) {
        boolean useTangential = !auto.wantsAutoSort() || flags.contains(ShootPathFlag.LAST);
        PathBuilder builder = PathUtil.addPathBuilderLine(robot, startPoseBlue, lastPath, getShootPose(ShootPathType.EDGE, flags), mirror, useTangential, useTangential);
        if (!auto.wantsAutoSort()) {
            builder = builder.setConstraintsForLast(RELAXED_CONSTRAINTS);
        }
        lastPath = builder.build();
        return lastPath;
    }

    private PathChain intakeSpike1Path() {
        this.lastPath = PathUtil.addPathBuilderCurve(robot, startPoseBlue, lastPath, INTAKE_1_CONTROL, INTAKE_1_POSE, mirror, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
        return lastPath;
    }

    private PathChain intakeSpike2Path() {
        this.lastPath = PathUtil.addPathBuilderCurve(robot, startPoseBlue, lastPath, INTAKE_2_CONTROL, INTAKE_2_POSE, mirror, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
        return lastPath;
    }

    private PathChain prepareIntakeSpike3Path() {
        // TODO: i think we can combine the two paths into one PathChain
        // it might be a little sus here that addPathBuilderCurve sets the heading interpolation to linear but we then override this
        Pose2d prepareIntakePose = PREPARE_INTAKE_3_POSE.mirror(mirror);
        this.lastPath = PathUtil.addPathBuilderCurve(robot, startPoseBlue, lastPath, INTAKE_3_CONTROL, PREPARE_INTAKE_3_POSE, mirror, false, false)
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(0.0, 0.4, HeadingInterpolator.tangent),
                                new HeadingInterpolator.PiecewiseNode(0.4, 0.7,
                                        FixedHeadingInterpolator.linearFromPoint(
                                                () -> robot.follower.getHeading(),
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
        return lastPath;
    }

    private PathChain intakeSpike3Path() {
        this.lastPath = PathUtil.addPathBuilderLine(robot, startPoseBlue, lastPath, INTAKE_3_POSE, mirror, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
        return lastPath;
    }

    // This is for pushing the gate after a SPIKE STRIP, not for cycling gate intake.
    //
    // preparePushGatePath and pushGatePath should be used together.
    private PathChain preparePushGatePath() {
        // TODO: i think we can combine the two paths into one PathChain
        this.lastPath = PathUtil.addPathBuilderLine(robot, lastPath, PREPARE_PUSH_GATE_POSE, mirror, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
        return lastPath;
    }

    // This is for pushing the gate after a SPIKE STRIP, not for cycling gate intake.
    private PathChain pushGatePath() {
        this.lastPath = PathUtil.addPathBuilderLine(robot, lastPath, PUSH_GATE_POSE, mirror, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
        return lastPath;
    }

    // This is for CYCLING gate intake, not for pushing the gate after a spike strip.
    private PathChain intakeGatePath1() {
//        this.lastPath = PathUtil.addPathBuilderCurve(robot, startPoseBlue, lastPath, GATE_CONTROL_POSE, AFTER_GATE, mirror, false, false)
//                .setBrakingStrength(0.6)
//                .build();
        this.lastPath = PathUtil.addPathBuilderCurve(robot, startPoseBlue, lastPath, GATE_CONTROL_POSE, HITTING_GATE, mirror, false, false)
                .setBrakingStrength(0.6)
                .build();
        return lastPath;
    }

    private PathChain intakeGatePath2() {
        this.lastPath = PathUtil.addPathBuilderCurve(robot, startPoseBlue, lastPath, GATE_CONTROL_POSE_2, AFTER_GATE, mirror, false, false)
                .setConstantHeadingInterpolation(AFTER_GATE.mirror(mirror).heading)
                .setBrakingStrength(0.6)
                .build();
        return lastPath;
    }

    private PathChain shootGatePath(EnumSet<ShootPathFlag> flags) {
        this.lastPath = PathUtil.addPathBuilderLine(robot, startPoseBlue, lastPath, getShootPose(ShootPathType.EDGE, flags), mirror, true, true)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
        return lastPath;
    }

    public PathChain customPath(PathChain path) {
        return this.lastPath = path;
    }

    // NB: shootCommand calls GoToIntakeStateCommand if not the last path.
    private Command shootCommand(EnumSet<ShootPathFlag> flags) {
        Command command = new ShootThreeBallsCommand(robot);
        if (auto.wantsAutoSort()) {
            command = command.andThen(new SequentialCommandGroup(
                    new WaitForSpindexerYawCommand(robot.spindexer).withTimeout(500),
                    new WaitCommand(SHOOT_DELAY)
            ));
        }

        if (!flags.contains(ShootPathFlag.LAST)) {
            command = command.andThen(
                    new GoToRestingStateCommand(robot),
                    new WaitUntilCommand(() -> robot.spindexer.isWallDown()),
                    new GoToIntakeStateCommand(robot),
                    new LogCatCommand("AutoBuilder", "ending shoot")
            );
        }

        if (flags.contains(ShootPathFlag.EARLY_LEAVE)) {
            command = new SequentialCommandGroup(
                    new ScheduleCommand(command),
                    new WaitCommand(600)
            );
        }
        return command.andThen(new IncrementNumCyclesCommand(auto));
    }

    // For NEAR ZONE preload shooting.
    public Command shootPreload(ShootPathFlag... flagArr) {
        boolean wantsAutoSort = robot.getAutoSort();
        EnumSet<ShootPathFlag> flags = ArrayUtil.toEnumSet(flagArr, ShootPathFlag.class);
        PathChain path = shootPreloadPath(flags);
        path.lastPath().setBrakingStrength(shootBrakingStrength);
        if (wantsAutoSort) {
            return new SequentialCommandGroup(
                    new FollowPathCommand(robot.follower, path, true),
                    new WaitCommand(PRELOAD_PRE_SHOOT_DELAY),
                    new WaitForGlyphCommand(robot.camera).withTimeout(AutoConstants.WAIT_TIMEOUT_MOTIF),

                    // `disableAprilTagsAfterGlyph` handles main disabling logic, this is
                    // just a fallback.
                    new InstantCommand(() -> robot.camera.setAprilTagsEnabled(false)),
                    new PrepareShootCommand(robot),
                    shootCommand(flags)
            );
        } else {
            // No need to PrepareShootCommand here; since init will do it for us.
            if (flags.contains(ShootPathFlag.SOTM)) {
                return new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitForFlywheelCommand(robot.shooter).withTimeout(625),
                                new WaitCommand(250),
                                shootCommand(flags)
                        ),
                        new FollowPathCommand(robot.follower, path, false)
                );
            } else {
                return new SequentialCommandGroup(
                        new FollowPathCommand(robot.follower, path, false),
                        new WaitCommand(PRELOAD_PRE_SHOOT_DELAY),
                        shootCommand(flags)
                );
            }
        }
    }

    public Command shootPreloadFar(ShootPathFlag... flagArr) {
        EnumSet<ShootPathFlag> flags = ArrayUtil.toEnumSet(flagArr, ShootPathFlag.class);
        this.lastPath = PathUtil.addPathBuilderLine(robot, startPoseBlue, lastPath, SHOOT_FAR_POSE, mirror, false, false)
                .build();
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, lastPath, true),
                        new SequentialCommandGroup(
                                new WaitForFlywheelCommand(robot.shooter).withTimeout(PRELOAD_FAR_PRE_SHOOT_DELAY),
                                new WaitCommand(250)
                        )
                ),
                shootCommand(flags),
                new StopScanningForGlyphsCommand(robot.camera)
        );
    }

    private Command intakeSpike1() {
        PathChain path = intakeSpike1Path();
        Command followPathCommand = new FollowPathAndWaitForWallCommand(robot, path, true, MAX_DRIVETRAIN_POWER_INTAKING, 3.0);
        if (!auto.wantsAutoSort()) {
            followPathCommand = followPathCommand.raceWith(new WaitForIntakeCommand(robot));
        }
        return new SequentialCommandGroup(
                new GoToIntakeStateCommand(robot),
                followPathCommand,
                new WaitForIntakeCommand(robot).withTimeout(INTAKE_DELAY),
                new SetIntakeSpeedCommand(robot.intake, 0)
        );
    }

    private Command intakeSpike2() {
        PathChain path = intakeSpike2Path();
        Command followPathCommand = new FollowPathAndWaitForWallCommand(robot, path, true, MAX_DRIVETRAIN_POWER_INTAKING, 18.0);
        if (!auto.wantsAutoSort()) {
            followPathCommand = followPathCommand.raceWith(new WaitForIntakeCommand(robot));
        }
        return new SequentialCommandGroup(
                new GoToIntakeStateCommand(robot),
                followPathCommand,
                new WaitForIntakeCommand(robot).withTimeout(INTAKE_DELAY),
                new SetIntakeSpeedCommand(robot.intake, 0)
        );
    }

    private Command intakeSpike3() {
        return new SequentialCommandGroup(
                new GoToIntakeStateCommand(robot),
                new FollowPathAndWaitForWallCommand(robot, prepareIntakeSpike3Path(), true, MAX_DRIVETRAIN_POWER_INTAKING, 36.0),
                new FollowPathCommand(robot.follower, intakeSpike3Path(), true),
                new WaitForIntakeCommand(robot).withTimeout(INTAKE_DELAY),
                new SetIntakeSpeedCommand(robot.intake, 0)
        );
    }

    /**
     * Command to intake from spike number.
     *
     * @param spikeNumber The spike number to intake from, where 1 is the closest to the goal and 3 is closest to the human player.
     * @return The command to intake from the specified spike.
     */
    public Command intakeSpike(int spikeNumber) {
        switch (spikeNumber) {
            case 1:
                return intakeSpike1();
            case 2:
                return intakeSpike2();
            case 3:
                return intakeSpike3();
            default:
                throw new IllegalArgumentException("Invalid spike number: " + spikeNumber);
        }
    }

    public Command intakeSpikeHorizontal(int spikeNumber) {
        Pose2d intakePose;
        if (spikeNumber == 1) {
            intakePose = INTAKE_1_HORIZ_POSE;
        } else if (spikeNumber == 2) {
            intakePose = INTAKE_2_HORIZ_POSE;
        } else if (spikeNumber == 3) {
            intakePose = INTAKE_3_HORIZ_POSE;
        } else {
            throw new IllegalArgumentException("Invalid spike number: " + spikeNumber);
        }
        this.lastPath = PathUtil.addPathBuilderCurve(robot, startPoseBlue, lastPath, INTAKE_BEFORE_HORIZ_CONTROL, INTAKE_BEFORE_HORIZ_POSE, mirror, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .addPath(
                        new BezierLine(
                                INTAKE_BEFORE_HORIZ_POSE.mirror(mirror).toPedro(),
                                intakePose.mirror(mirror).toPedro()
                        )
                )
                .setConstantHeadingInterpolation(INTAKE_BEFORE_HORIZ_POSE.mirror(mirror).heading)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
        return new SequentialCommandGroup(
                new FollowPathCommand(robot.follower, lastPath, true, 0.9),
                new WaitForIntakeCommand(robot).withTimeout(INTAKE_DELAY_HORIZ)
        );
    }

    private Command createFollowShootPathAndShootCommand(long prepareShootDelay, PathChain shootPath, EnumSet<ShootPathFlag> flags) {
        // Use AtomicBoolean here since Java lambdas capture by value.
        AtomicBoolean hasFinishedPath = new AtomicBoolean(false);
        double distanceConstraint = flags.contains(ShootPathFlag.EARLY_SHOOT) ? EARLY_SHOOT_DISTANCE : 0.0;
        long shootDelay = auto.wantsAutoSort() ? AutoConstants.SORTED_SHOOT_DELAY : 0;
        boolean holdEnd = auto.wantsAutoSort();
        shootPath.lastPath().setBrakingStrength(shootBrakingStrength);

        // TODO: possible race condition
        Supplier<Boolean> hasStartedPrepareShoot = () -> robot.robotState.equals(RobotState.READY_TO_SHOOT) || robot.robotState.equals(RobotState.TRANSFER);
        Supplier<Command> maybePrepareShootCommand = () -> new ConditionalCommand(
                new PrepareShootCommand(robot, prepareShootTimeBeforeReverseIntake, true),
                new InstantCommand(() -> {}),
                () -> !(robot.robotState.equals(RobotState.READY_TO_SHOOT) || robot.robotState.equals(RobotState.TRANSFER))
        );

        // Although this is a ParallelCommandGroup, we essentially implement
        // custom Sequential logic using the AtomicBoolean flags.
        return new ParallelCommandGroup(
                // Follow path
                new SequentialCommandGroup(
                        new FollowPathCommand(robot.follower, shootPath, holdEnd),
                        new InstantCommand(() -> hasFinishedPath.set(true))
                )
                .andThen(new LogCatCommand("AutoBuilder", "follow path done, waiting for prepare shoot")),

                // Prepare shoot
                new SequentialCommandGroup(
                        new WaitUntilCommand(() ->
                                !auto.wantsAutoSort()
                                || hasStartedPrepareShoot.get()
                                || hasFinishedPath.get()
                                || !ArrayUtil.contains(robot.spindexer.getBallPositions(), BallColor.NONE)
                        ).withTimeout(prepareShootDelay),
                        maybePrepareShootCommand.get()
                )
                .andThen(new LogCatCommand("AutoBuilder", "prepare shoot done, waiting for path or distance")),

                // Shoot command
                new SequentialCommandGroup(
                        new WaitUntilCommand(() -> robot.robotState.equals(RobotState.READY_TO_SHOOT)),
                        new WaitUntilCommand(() ->
                                hasFinishedPath.get() || robot.follower.getDistanceRemaining() < distanceConstraint
                        ),
                        new WaitCommand(shootDelay),
                        shootCommand(flags)
                )
                .andThen(new LogCatCommand("AutoBuilder", "shoot command done"))
        ).andThen(new LogCatCommand("AutoBuilder", "createFollow end"));
    }

    /**
     * Command to shoot from spike number.
     *
     * @param spikeNumber The spike number to intake from, where 1 is the closest to the goal and 3 is closest to the human player.
     * @param flagArr     Whether this is the last shoot command in the auto sequence.
     * @return The command to intake from the specified spike.
     */
    public Command shootSpike(int spikeNumber, ShootPathFlag... flagArr) {
        EnumSet<ShootPathFlag> flags = ArrayUtil.toEnumSet(flagArr, ShootPathFlag.class);
        PathChain path;
        if (flags.contains(ShootPathFlag.PRELOAD_SHOOT_SPOT)) {
            path = shootPreloadPath(flags);
        } else {
            path = shootSpikePath(flags);
        }
        Command endCommand = new InstantCommand(() -> {});
        if (flags.contains(ShootPathFlag.LAST) && auto.wantsAutoSort()) {
            endCommand = parkSorted();
        }
        return new SequentialCommandGroup(
                createFollowShootPathAndShootCommand(waitBeforeShooting, path, flags),
                endCommand
        );
    }

    public Command cycleSpike(int spikeNumber, ShootPathFlag... flags) {
        return new SequentialCommandGroup(
                intakeSpike(spikeNumber),
                shootSpike(spikeNumber, flags)
        );
    }

    public Command parkSorted(){
        this.lastPath = PathUtil.addPathBuilderLine(robot, startPoseBlue, lastPath, SHOOT_LAST_POSE, mirror, false, false).build();

        return new FollowPathCommand(robot.follower, this.lastPath, true);
    }

    // For pushing the gate after a SPIKE STRIP. Not for cycling gate intake.
    public Command pushGate() {
        return new SequentialCommandGroup(
                new FollowPathCommand(robot.follower, preparePushGatePath(), true, MAX_DRIVETRAIN_POWER_INTAKING),
                new FollowPathCommand(robot.follower, pushGatePath(), true, MAX_DRIVETRAIN_POWER_INTAKING)
        );
    }

    public Command intakeGate() {
        return new SequentialCommandGroup(
                new FollowPathCommand(robot.follower, intakeGatePath1(), true, MAX_DRIVETRAIN_POWER_INTAKING),
                new WaitCommand(250),
                new FollowPathCommand(robot.follower, intakeGatePath2(), true, MAX_DRIVETRAIN_POWER_INTAKING),
                new WaitForIntakeCommand(robot).withTimeout(GATE_INTAKE_DELAY)
        );
    }

    public Command shootGate(boolean reverseIntake, ShootPathFlag... flagArr) {
        EnumSet<ShootPathFlag> flags = ArrayUtil.toEnumSet(flagArr, ShootPathFlag.class);
        return new ParallelCommandGroup(
                createFollowShootPathAndShootCommand(250, shootGatePath(flags), flags),
                new SequentialCommandGroup(
                        new WaitCommand(REVERSE_INTAKE_GATE_DELAY),
                        new ConditionalCommand(
                                new SetIntakeSpeedCommand(robot.intake, IntakeSubsystem.REVERSE_SPEED),
                                new InstantCommand(() -> {}),
                                // Only reverse if reverseIntake and we get 3 balls
                                () -> reverseIntake && !ArrayUtil.contains(robot.spindexer.getBallPositions(), BallColor.NONE)
                        )
                )
        );
    }

    public Command cycleGate(boolean reverseIntake, ShootPathFlag... flags) {
        return new SequentialCommandGroup(
                intakeGate(),
                shootGate(reverseIntake, flags)
        );
    }

    public Command intakeSpike3Far() {
        this.lastPath = PathUtil.addPathBuilderCurve(robot, startPoseBlue, lastPath, INTAKE_3_CONTROL_FAR, INTAKE_3_POSE, mirror, false, false)
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(0.0, 0.25,
                                        HeadingInterpolator.linear(lastPath.getFinalHeadingGoal(), INTAKE_3_POSE.mirror(mirror).heading)
                                ),
                                new HeadingInterpolator.PiecewiseNode(0.25, 0.8,
                                        HeadingInterpolator.constant(INTAKE_3_POSE.mirror(mirror).heading)
                                ),
                                new HeadingInterpolator.PiecewiseNode(0.8, 1.0,
                                        FixedHeadingInterpolator.linear(
                                                INTAKE_3_POSE.mirror(mirror).heading,
                                                INTAKE_3_POSE_FAR.mirror(mirror).heading,
                                                0.8, 1.0
                                        )
                                )
                        )
                )
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
        return new SequentialCommandGroup(
                new ParallelRaceGroup(
                    new FollowPathAndWaitForWallCommand(robot, lastPath, true, MAX_DRIVETRAIN_POWER_INTAKING, 12.0),
                    new WaitForIntakeCommand(robot)
                ),
                new WaitForIntakeCommand(robot).withTimeout(INTAKE_DELAY)
        );
    }

    public Command shootSpike3Far(ShootPathFlag ...flagArr) {
        EnumSet<ShootPathFlag> flags = ArrayUtil.toEnumSet(flagArr, ShootPathFlag.class);
        this.lastPath = PathUtil.addPathBuilderLine(robot, startPoseBlue, lastPath, SHOOT_FAR_POSE, mirror, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
        return createFollowShootPathAndShootCommand(250, lastPath, flags);
    }

    public Command intakeWall(boolean reverseIntake) {
        this.lastPath = PathUtil.addPathBuilderLine(robot, startPoseBlue, lastPath, INTAKE_WALL_POSE, mirror, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .setNoDeceleration()
                .build();
        return new SequentialCommandGroup(
                new ParallelRaceGroup(
                        new FollowPathAndWaitForWallCommand(robot, lastPath, true, 1.0, 12.0),
                        new WaitForIntakeCommand(robot)
                ),
                new WaitForIntakeCommand(robot).withTimeout(WALL_INTAKE_DELAY),
                new ConditionalCommand(
                        new SetIntakeSpeedCommand(robot.intake, IntakeSubsystem.REVERSE_SPEED),
                        new InstantCommand(() -> {}),
                        // Only reverse if reverseIntake and we get 3 balls
                        () -> reverseIntake && !ArrayUtil.contains(robot.spindexer.getBallPositions(), BallColor.NONE)
                )
        );
    }

    public Command prepareVision(){
        this.lastPath = PathUtil.addPathBuilderLine(robot, startPoseBlue, lastPath, VISION_POSE, mirror, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .setNoDeceleration()
                .build();

        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.camera.setBallPipelineEnabled(true)),
                        new FollowPathCommand(robot.follower, lastPath, true, 0.9)
                ),
                new WaitUntilCommand(() -> robot.camera.hasBlob())
        );

    }

    public Command intakeVision(boolean reverseIntake){
        this.lastPath = PathUtil.addPathBuilderLine(robot, startPoseBlue, lastPath, robot.camera.getBallCoords(), mirror, true, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .setNoDeceleration()
                .build();
        return new SequentialCommandGroup(
                new InstantCommand(() -> robot.camera.setBallPipelineEnabled(false)),
                new FollowPathCommand(robot.follower, lastPath, true),
                new WaitForIntakeCommand(robot).withTimeout(WALL_INTAKE_DELAY),
                new ConditionalCommand(
                        new SetIntakeSpeedCommand(robot.intake, IntakeSubsystem.REVERSE_SPEED),
                        new InstantCommand(() -> {}),
                        // Only reverse if reverseIntake and we get 3 balls
                        () -> reverseIntake && !ArrayUtil.contains(robot.spindexer.getBallPositions(), BallColor.NONE)
                ),
                new InstantCommand(() -> {
                    robot.camera.resetBlob();
                    robot.camera.setBallPipelineEnabled(false);
                })
        );

    }


    public Command intakeTunnel(boolean reverseIntake) {
        this.lastPath = PathUtil.addPathBuilderLine(robot, startPoseBlue, lastPath, INTAKE_TUNNEL_POSE, mirror, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .setNoDeceleration()
                .build();
        return new SequentialCommandGroup(
                new ParallelRaceGroup(
                        new FollowPathAndWaitForWallCommand(robot, lastPath, true, 1.0, 20.0),
                        new WaitForIntakeCommand(robot)
                ),
                new WaitForIntakeCommand(robot).withTimeout(WALL_INTAKE_DELAY),
                new ConditionalCommand(
                        new SetIntakeSpeedCommand(robot.intake, IntakeSubsystem.REVERSE_SPEED),
                        new InstantCommand(() -> {}),
                        // Only reverse if reverseIntake and we get 3 balls
                        () -> reverseIntake && !ArrayUtil.contains(robot.spindexer.getBallPositions(), BallColor.NONE)
                )
        );
    }

    public Command shootWall( ShootPathFlag ...flagArr) {
        EnumSet<ShootPathFlag> flags = ArrayUtil.toEnumSet(flagArr, ShootPathFlag.class);
        this.lastPath = PathUtil.addPathBuilderLine(robot, startPoseBlue, lastPath, SHOOT_FAR_POSE, mirror, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
        return createFollowShootPathAndShootCommand(250, lastPath, flags);
    }

    public Command cycleTunnel(boolean reverseIntake, ShootPathFlag ...flagArr) {
        return new SequentialCommandGroup(
                intakeTunnel(reverseIntake),
                shootWall(flagArr)
        );
    }

    public Command cycleWall(boolean reverseIntake, ShootPathFlag ...flagArr) {
        return new SequentialCommandGroup(
                intakeWall(reverseIntake),
                shootWall(flagArr)
        );
    }

    //when running this the first time run prepare vision seperatly first
    public Command cycleVision(boolean reverseIntake, ShootPathFlag... flagArr){
        return new DeferredCommand(() -> new SequentialCommandGroup(
                intakeVision(reverseIntake),
                shootWall(flagArr),
                prepareVision()
        ), null);
    }
    public Command intakeWallLong(){
        this.lastPath = PathUtil.addPathBuilderLine(robot, startPoseBlue, lastPath, INTAKE_WALL_POSE, mirror, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .setNoDeceleration()
                .build();
        return new SequentialCommandGroup(
                new GoToIntakeStateCommand(robot),
                new FollowPathCommand(robot.follower, lastPath, true)
        );
    }
    public Command controlPathLong(){
        this.lastPath = PathUtil.addPathBuilderCurve(robot, startPoseBlue, lastPath, CONTROL_POSE_LONG_INTAKE, START_POSE_LONG_INTAKE, mirror, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .setNoDeceleration()
                .build();
        return new SequentialCommandGroup(
                new FollowPathCommand(robot.follower, this.lastPath)
        );
    }
    public Command intakeTunnelLong(){
        this.lastPath = PathUtil.addPathBuilderLine(robot, startPoseBlue, lastPath, END_POSE_LONG_INTAKE, mirror, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .setNoDeceleration()
                .build();
        return new SequentialCommandGroup(
                new FollowPathCommand(robot.follower, this.lastPath)
        );
    }
    public Command shootWallLong(ShootPathFlag ...flagArr){
        EnumSet<ShootPathFlag> flags = ArrayUtil.toEnumSet(flagArr, ShootPathFlag.class);
        this.lastPath = PathUtil.addPathBuilderLine(robot, startPoseBlue, lastPath, SHOOT_FAR_POSE, mirror, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(0, 0.7,
                                        FixedHeadingInterpolator.linearFromPoint(
                                                () -> robot.follower.getHeading(),
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
        return createFollowShootPathAndShootCommand(250, lastPath, flags);

    }

    public Command cycleLong(boolean reverseIntake, ShootPathFlag... flagArr){
        return new SequentialCommandGroup(
                new ParallelRaceGroup(
                        new SequentialCommandGroup(
                                intakeWallLong(),
                                controlPathLong(),
                                intakeTunnelLong()
                        ),
                        new WaitForIntakeCommand(robot)
                ),
                new ConditionalCommand(
                        new SetIntakeSpeedCommand(robot.intake, IntakeSubsystem.REVERSE_SPEED),
                        new InstantCommand(() -> {}),
                        // Only reverse if reverseIntake and we get 3 balls
                        () -> reverseIntake && !ArrayUtil.contains(robot.spindexer.getBallPositions(), BallColor.NONE)
                ),
                shootWallLong(flagArr)


        );
    }

}
