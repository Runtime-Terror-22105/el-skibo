package org.firstinspires.ftc.teamcode.robot.auto;

import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.AFTER_GATE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.GATE_CONTROL_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.GATE_INTAKE_DELAY;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_1_CONTROL;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_1_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_2_CONTROL;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_2_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_3_CONTROL;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_3_CONTROL_FAR;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_3_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_DELAY;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_WALL_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.MAX_DRIVETRAIN_POWER_INTAKING;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.PRELOAD_FAR_PRE_SHOOT_DELAY;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.PRELOAD_PRE_SHOOT_DELAY;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.PREPARE_INTAKE_3_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.PREPARE_PUSH_GATE_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.PRE_SHOOT_DELAY;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.PUSH_GATE_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.RELAXED_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_DELAY;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_EDGE_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_FAR_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_LAST_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_PRELOAD_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.WALL_INTAKE_DELAY;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.pedroPathing.FixedHeadingInterpolator;
import org.firstinspires.ftc.teamcode.robot.command.WaitForIntakeCommand;
import org.firstinspires.ftc.teamcode.robot.command.shooter.ShootThreeBallsCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.PrepareShootCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.WaitForSpindexerYawCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToIntakeStateCommand;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.util.StartConfig;

@Config
public class AutoBuilder {
    public final Pose2d startPoseBlue;
    public final Robot robot;
    public final boolean mirror;
    private PathChain lastPath = null;

    public AutoBuilder(Robot robot, Team team, StartConfig initial) {
        this.robot = robot;
        // NB: We do not mirror the start pose here because the path builder's mirror parameter
        // will handle it for us.
        this.startPoseBlue = initial.getStartPoseBlue();
        this.mirror = Team.RED.equals(team);
    }

    private static Pose2d getShootPose(Pose2d mainPose, boolean isLast) {
        return isLast ? SHOOT_LAST_POSE : mainPose;
    }

    public PathChain getLastPath() {
        return lastPath;
    }

    private PathChain shootPreloadPath(boolean isLast) {
        this.lastPath = PathUtil.addPathBuilderLine(robot, startPoseBlue, lastPath, getShootPose(SHOOT_PRELOAD_POSE, isLast), mirror, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
        return lastPath;
    }

    private PathChain intakeSpike1Path() {
        this.lastPath = PathUtil.addPathBuilderCurve(robot, startPoseBlue, lastPath, INTAKE_1_CONTROL, INTAKE_1_POSE, mirror, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
        return lastPath;
    }

    private PathChain shootSpike1Path(boolean isLast) {
        this.lastPath = PathUtil.addPathBuilderLine(robot, startPoseBlue, lastPath, getShootPose(SHOOT_EDGE_POSE, isLast), mirror, true, true)
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

    private PathChain shootSpike2Path(boolean isLast) {
        this.lastPath = PathUtil.addPathBuilderLine(robot, startPoseBlue, lastPath, getShootPose(SHOOT_EDGE_POSE, isLast), mirror, true, true)
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

    private PathChain shootSpike3Path(boolean isLast) {
        this.lastPath = PathUtil.addPathBuilderLine(robot, startPoseBlue, lastPath, getShootPose(SHOOT_EDGE_POSE, isLast), mirror, true, true)
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
    private PathChain intakeGatePath() {
        this.lastPath = PathUtil.addPathBuilderCurve(robot, startPoseBlue, lastPath, GATE_CONTROL_POSE, AFTER_GATE, mirror, false, false)
                .setBrakingStrength(0.6)
                .build();
        return lastPath;
    }

    private PathChain shootGatePath(boolean isLast) {
        this.lastPath = PathUtil.addPathBuilderLine(robot, startPoseBlue, lastPath, getShootPose(SHOOT_EDGE_POSE, isLast), mirror, true, true)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
        return lastPath;
    }

    public PathChain customPath(PathChain path) {
        return this.lastPath = path;
    }

    // For NEAR ZONE preload shooting.
    public Command shootPreload(boolean isLast) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new PrepareShootCommand(robot),
                        new FollowPathCommand(robot.follower, shootPreloadPath(isLast), false)
                ),
                new WaitCommand(PRELOAD_PRE_SHOOT_DELAY),
                new ShootThreeBallsCommand(robot),
                new WaitForSpindexerYawCommand(robot.spindexer).withTimeout(500),
//                new InstantCommand(() -> robot.camera.stopScanningForGlyphs()),
                new WaitCommand(SHOOT_DELAY)
        );
    }

    // For NEAR ZONE preload shooting.
    public Command shootPreload() {
        return shootPreload(false);
    }

    public Command shootPreloadFar() {
        this.lastPath = PathUtil.addPathBuilderLine(robot, startPoseBlue, lastPath, SHOOT_FAR_POSE, mirror, false, false)
                .build();
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new PrepareShootCommand(robot),
                        new FollowPathCommand(robot.follower, lastPath, false),
                        new WaitCommand(PRELOAD_FAR_PRE_SHOOT_DELAY)
                ),
                new ShootThreeBallsCommand(robot),
                new WaitForSpindexerYawCommand(robot.spindexer).withTimeout(500),
//                new InstantCommand(() -> robot.camera.stopScanningForGlyphs()),
                new WaitCommand(SHOOT_DELAY)
        );
    }

    private Command intakeSpike1() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, intakeSpike1Path(), true, MAX_DRIVETRAIN_POWER_INTAKING),
                        new GoToIntakeStateCommand(robot)
                ),
                new WaitForIntakeCommand(robot).withTimeout(INTAKE_DELAY)
        );
    }

    private Command intakeSpike2() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, intakeSpike2Path(), true, MAX_DRIVETRAIN_POWER_INTAKING),
                        new GoToIntakeStateCommand(robot)
                ),
                new WaitForIntakeCommand(robot).withTimeout(INTAKE_DELAY)
        );
    }

    private Command intakeSpike3() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, prepareIntakeSpike3Path(), true, MAX_DRIVETRAIN_POWER_INTAKING),
                        new GoToIntakeStateCommand(robot)
                ),
                new FollowPathCommand(robot.follower, intakeSpike3Path(), true),
                new WaitForIntakeCommand(robot).withTimeout(INTAKE_DELAY)
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

    /**
     * Command to shoot from spike number.
     *
     * @param spikeNumber The spike number to intake from, where 1 is the closest to the goal and 3 is closest to the human player.
     * @param isLast      Whether this is the last shoot command in the auto sequence.
     * @return The command to intake from the specified spike.
     */
    public Command shootSpike(int spikeNumber, boolean isLast) {
        PathChain shootPath;
        switch (spikeNumber) {
            case 1:
                shootPath = shootSpike1Path(isLast);
                break;
            case 2:
                shootPath = shootSpike2Path(isLast);
                break;
            case 3:
                shootPath = shootSpike3Path(isLast);
                break;
            default:
                throw new IllegalArgumentException("Invalid spike number: " + spikeNumber);
        }

        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, shootPath, false),
                        new WaitCommand(250).andThen(new PrepareShootCommand(robot))
                ),
                new WaitCommand(PRE_SHOOT_DELAY),
                new ShootThreeBallsCommand(robot),
                new WaitForSpindexerYawCommand(robot.spindexer).withTimeout(2000),
                new WaitCommand(SHOOT_DELAY)
        );
    }

    public Command shootSpike(int spikeNumber) {
        return shootSpike(spikeNumber, false);
    }

    public Command cycleSpike(int spikeNumber, boolean isLast) {
        return new SequentialCommandGroup(
                intakeSpike(spikeNumber),
                shootSpike(spikeNumber, isLast)
        );
    }

    public Command cycleSpike(int spikeNumber) {
        return cycleSpike(spikeNumber, false);
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
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, intakeGatePath(), true, MAX_DRIVETRAIN_POWER_INTAKING),
                        new GoToIntakeStateCommand(robot)
                ),
                new WaitForIntakeCommand(robot).withTimeout(GATE_INTAKE_DELAY)
        );
    }

    public Command shootGate(boolean isLast) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, shootGatePath(isLast), false),
                        new WaitCommand(250).andThen(new PrepareShootCommand(robot, true))
                ),
                new WaitCommand(PRE_SHOOT_DELAY),
                new ShootThreeBallsCommand(robot),
                new WaitForSpindexerYawCommand(robot.spindexer).withTimeout(2000),
                new WaitCommand(SHOOT_DELAY)
        );
    }

    public Command shootGate() {
        return shootGate(false);
    }

    public Command cycleGate(boolean isLast) {
        return new SequentialCommandGroup(
                intakeGate(),
                shootGate(isLast)
        );
    }

    public Command cycleGate() {
        return cycleGate(false);
    }

    public Command intakeSpike3Far() {
        this.lastPath = PathUtil.addPathBuilderCurve(robot, startPoseBlue, lastPath, INTAKE_3_CONTROL_FAR, INTAKE_3_POSE, mirror, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, lastPath, true, MAX_DRIVETRAIN_POWER_INTAKING),
                        new GoToIntakeStateCommand(robot)
                ),
                new WaitForIntakeCommand(robot).withTimeout(INTAKE_DELAY)
        );
    }

    public Command shootSpike3Far() {
        this.lastPath = PathUtil.addPathBuilderLine(robot, startPoseBlue, lastPath, SHOOT_FAR_POSE, mirror, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, lastPath, false),
                        new WaitCommand(250).andThen(new PrepareShootCommand(robot))
                ),
                new WaitCommand(PRE_SHOOT_DELAY),
                new ShootThreeBallsCommand(robot),
                new WaitForSpindexerYawCommand(robot.spindexer).withTimeout(2000),
                new WaitCommand(SHOOT_DELAY)
        );
    }

    public Command intakeWall() {
        this.lastPath = PathUtil.addPathBuilderLine(robot, startPoseBlue, lastPath, INTAKE_WALL_POSE, mirror, false, false)
                .build();
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, lastPath, true, MAX_DRIVETRAIN_POWER_INTAKING),
                        new GoToIntakeStateCommand(robot)
                ),
                new WaitForIntakeCommand(robot).withTimeout(WALL_INTAKE_DELAY)
        );
    }

    public Command shootWall() {
        this.lastPath = PathUtil.addPathBuilderLine(robot, startPoseBlue, lastPath, SHOOT_FAR_POSE, mirror, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, lastPath, false),
                        new WaitCommand(250).andThen(new PrepareShootCommand(robot))
                ),
                new WaitCommand(PRE_SHOOT_DELAY),
                new ShootThreeBallsCommand(robot),
                new WaitForSpindexerYawCommand(robot.spindexer).withTimeout(2000),
                new WaitCommand(SHOOT_DELAY)
        );
    }

    public Command cycleWall() {
        return new SequentialCommandGroup(
                intakeWall(),
                shootWall()
        );
    }
}
