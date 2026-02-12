package org.firstinspires.ftc.teamcode.robot.auto;

import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.AFTER_GATE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.GATE_CONTROL_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.GATE_INTAKE_DELAY;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.GATE_INTAKE_TIMEOUT;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_1_CONTROL;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_1_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_2_CONTROL;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_2_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_3_CONTROL;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_DELAY;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.MAX_DRIVETRAIN_POWER_INTAKING;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.PRELOAD_PRE_SHOOT_DELAY;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.PREPARE_INTAKE_3_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.PRE_SHOOT_DELAY;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.RELAXED_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_DELAY;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_EDGE_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_LAST_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_PRELOAD_POSE;

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
    private final Pose2d startPoseBlue;
    private final Robot robot;
    private final boolean mirror;
    private PathChain lastPath = null;

    public boolean finished;

    public AutoBuilder(Robot robot, Team team, StartConfig initial) {
        this.robot = robot;
        // NB: We do not mirror the start pose here because the path builder's mirror parameter
        // will handle it for us.
        this.startPoseBlue = initial.getStartPoseBlue();
        this.mirror = Team.RED.equals(team);

        // Team.getStartPose will mirror the pose for us if necessary.
        robot.follower.setStartingPose(team.getStartPose(initial).toPedro());
        robot.goalPos = team.getGoalPos();
    }

    public boolean hasFinished() {
        return this.finished;
    }

    public void setFinished() {
        this.finished = true;
    }

    private static Pose2d getShootPose(boolean isLast) {
        return isLast ? SHOOT_LAST_POSE : SHOOT_EDGE_POSE;
    }

    public PathChain shootPreloadPath() {
        this.lastPath = PathUtil.addPathBuilderLine(robot, startPoseBlue, SHOOT_PRELOAD_POSE, mirror, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
        return lastPath;
    }

    public PathChain spike1IntakePath() {
        this.lastPath = PathUtil.addPathBuilderCurve(robot, startPoseBlue, lastPath, INTAKE_1_CONTROL, INTAKE_1_POSE, mirror, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
        return lastPath;
    }

    public PathChain spike1ShootPath(boolean isLast) {
        this.lastPath = PathUtil.addPathBuilderLine(robot, startPoseBlue, lastPath, getShootPose(isLast), mirror, true, true)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
        return lastPath;
    }

    public PathChain spike2IntakePath() {
        this.lastPath = PathUtil.addPathBuilderCurve(robot, startPoseBlue, lastPath, INTAKE_2_CONTROL, INTAKE_2_POSE, mirror, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
        return lastPath;
    }

    public PathChain spike2ShootPath(boolean isLast) {
        this.lastPath = PathUtil.addPathBuilderLine(robot, startPoseBlue, lastPath, getShootPose(isLast), mirror, true, true)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
        return lastPath;
    }

    public PathChain spike3IntakePath() {
        // it might be a little sus here that addPathBuilderCurve sets the heading interpolation to linear but we then override this
        this.lastPath = PathUtil.addPathBuilderCurve(robot, startPoseBlue, lastPath, INTAKE_3_CONTROL, PREPARE_INTAKE_3_POSE, mirror, false, false)
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(0.0, 0.4, HeadingInterpolator.tangent),
                                new HeadingInterpolator.PiecewiseNode(0.4, 1.0,
                                        FixedHeadingInterpolator.linearFromPoint(() -> robot.follower.getHeading(), PREPARE_INTAKE_3_POSE.heading, 0.4, 0.7)
                                )
                        )
                )
                .build();
        return lastPath;
    }

    public PathChain spike3ShootPath(boolean isLast) {
        this.lastPath = PathUtil.addPathBuilderLine(robot, startPoseBlue, lastPath, getShootPose(isLast), mirror, true, true)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
        return lastPath;
    }

    // This is for CYCLING gate intake, not for pushing the gate after a spike strip.
    public PathChain intakeGatePath() {
        this.lastPath = PathUtil.addPathBuilderCurve(robot, startPoseBlue, lastPath, GATE_CONTROL_POSE, AFTER_GATE, mirror, false, false)
                .setBrakingStrength(0.6)
                .build();
        return lastPath;
    }

    public PathChain shootGatePath(boolean isLast) {
        this.lastPath = PathUtil.addPathBuilderLine(robot, startPoseBlue, lastPath, getShootPose(isLast), mirror, true, true)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
        return lastPath;
    }

    public Command shootPreload() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new PrepareShootCommand(robot),
                        new FollowPathCommand(robot.follower, shootPreloadPath(), false)
                ),
                new WaitCommand(PRELOAD_PRE_SHOOT_DELAY),
                new ShootThreeBallsCommand(robot),
                new WaitForSpindexerYawCommand(robot.spindexer).withTimeout(500),
//                new InstantCommand(() -> robot.camera.stopScanningForGlyphs()),
                new WaitCommand(SHOOT_DELAY)
        );
    }

    /**
     * Command to intake from spike number.
     * @param spikeNumber The spike number to intake from, where 1 is the closest to the goal and 3 is closest to the human player.
     * @return The command to intake from the specified spike.
     */
    public Command spikeIntake(int spikeNumber) {
        PathChain intakePath;
        switch (spikeNumber) {
            case 1:
                intakePath = spike1IntakePath();
                break;
            case 2:
                intakePath = spike2IntakePath();
                break;
            case 3:
                intakePath = spike3IntakePath();
                break;
            default:
                throw new IllegalArgumentException("Invalid spike number: " + spikeNumber);
        }
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, intakePath, true, MAX_DRIVETRAIN_POWER_INTAKING),
                        new GoToIntakeStateCommand(robot)
                ),
                new WaitForIntakeCommand(robot).withTimeout(INTAKE_DELAY)
        );
    }

    /**
     * Command to shoot from spike number.
     * @param spikeNumber The spike number to intake from, where 1 is the closest to the goal and 3 is closest to the human player.
     * @param isLast Whether this is the last shoot command in the auto sequence.
     * @return The command to intake from the specified spike.
     */
    public Command spikeShoot(int spikeNumber, boolean isLast) {
        PathChain shootPath;
        switch (spikeNumber) {
            case 1:
                shootPath = spike1ShootPath(isLast);
                break;
            case 2:
                shootPath = spike2ShootPath(isLast);
                break;
            case 3:
                shootPath = spike3ShootPath(isLast);
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

    public Command spikeShoot(int spikeNumber) {
        return spikeShoot(spikeNumber, false);
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
}
