package org.firstinspires.ftc.teamcode.robot.auto;

import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.GATE_INTAKE_TIMEOUT;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_2_CONTROL;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_2_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_DELAY;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.MAX_DRIVETRAIN_POWER_INTAKING;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.PRE_SHOOT_DELAY;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.RELAXED_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_DELAY;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.command.WaitForIntakeCommand;
import org.firstinspires.ftc.teamcode.robot.command.shooter.ShootThreeBallsCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.PrepareShootCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.WaitForSpindexerYawCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToIntakeStateCommand;
import org.firstinspires.ftc.teamcode.robot.init.Robot;

import java.util.ArrayList;
import java.util.List;

import kotlin.NotImplementedError;

@Config
public class AutoBuilder {
    /**
     * Starting configurations for the robot.
     */
    public enum StartingConfiguration {
        NEAR,
        FAR
    }

    /**
     * Auto routine steps.
     */
    private enum AutoMove {
        PRELOAD,
        SPIKE_1,
        SPIKE_2,
        SPIKE_3,
        PARK_NEAR,
        PARK_FAR,
        GATE
    }

    private final Pose2d startPose;
    private final List<AutoMove> autoMoves;
    private final Robot robot;
    private final boolean mirror;
    private PathChain lastPath = null;

    public boolean finished;

    public AutoBuilder(Robot robot, Team team, StartingConfiguration initial) {
        this.robot = robot;
        this.startPose = getStartPose(team, initial);
        this.mirror = Team.RED.equals(team);

        robot.follower.setStartingPose(startPose.toPedro());
        robot.goalPos = team.getGoalPos();
        autoMoves = new ArrayList<>();
    }

    public boolean hasFinished() {
        return this.finished;
    }

    public void setFinished() {
        this.finished = true;
    }

    private static Pose2d getStartPose(Team team, StartingConfiguration initial) {
        if (initial == StartingConfiguration.NEAR) {
            return team.getStartPosNear();
        } else {
            return team.getStartPosFar();
        }
    }

    public PathChain spike2IntakePath() {
        PathChain path = PathUtil.addPathBuilderCurve(robot, startPose, lastPath, INTAKE_2_CONTROL, INTAKE_2_POSE, mirror, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
        this.lastPath = path;
        return path;
    }

    public Command preloadShootCommand() throws NotImplementedError {
        throw new NotImplementedError("sorry");
    }

    public Command spikeIntakeCommand() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, intake1Path, true, MAX_DRIVETRAIN_POWER_INTAKING),
                        new GoToIntakeStateCommand(robot)
                ),
                new WaitForIntakeCommand(robot).withTimeout(INTAKE_DELAY)
        );
    }

    public Command spikeShootCommand() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, shoot1Path, false),
                        new WaitCommand(250).andThen(new PrepareShootCommand(robot))
                ),
                new WaitCommand(PRE_SHOOT_DELAY),
                new ShootThreeBallsCommand(robot),
                new WaitForSpindexerYawCommand(robot.spindexer).withTimeout(2000),
                new WaitCommand(SHOOT_DELAY)
        );
    }

    public Command intakeGateCommand() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, hitGatePath, true, MAX_DRIVETRAIN_POWER_INTAKING),
                        new GoToIntakeStateCommand(robot)
                ),
                new WaitForIntakeCommand(robot).withTimeout(GATE_INTAKE_TIMEOUT)
        );
    }

    public Command shootGateCommand() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, gateToShootPath, false),
                        new WaitCommand(250).andThen(new PrepareShootCommand(robot, true))
                ),
                new WaitCommand(PRE_SHOOT_DELAY),
                new ShootThreeBallsCommand(robot),
                new WaitForSpindexerYawCommand(robot.spindexer).withTimeout(2000),
                new WaitCommand(SHOOT_DELAY)
        );
    }

    /**
     * Shoots the preload artifacts.
     * @return The AutoBuilder instance for chaining.
     */
    public AutoBuilder preload() {
        autoMoves.add(AutoMove.PRELOAD);
        return this;
    }

    /**
     * Intakes & shoots artifacts from spike mark 1.
     * @return The AutoBuilder instance for chaining.
     */
    public AutoBuilder spike1() {
        autoMoves.add(AutoMove.SPIKE_1);
        return this;
    }

    /**
     * Intakes & shoots artifacts from spike mark 2.
     * @return The AutoBuilder instance for chaining.
     */
    public AutoBuilder spike2() {
        autoMoves.add(AutoMove.SPIKE_2);
        return this;
    }

    /**
     * Intakes & shoots artifacts from spike mark 3.
     * @return The AutoBuilder instance for chaining.
     */
    public AutoBuilder spike3() {
        autoMoves.add(AutoMove.SPIKE_3);
        return this;
    }

    /**
     * Parks the robot in the near side parking area.
     * @return The AutoBuilder instance for chaining.
     */
    public AutoBuilder parkNear() {
        autoMoves.add(AutoMove.PARK_NEAR);
        return this;
    }

    /**
     * Parks the robot in the far side parking area.
     * @return The AutoBuilder instance for chaining.
     */
    public AutoBuilder parkFar() throws NotImplementedError {
        throw new NotImplementedError("mb parkFar is not implemented yet :(((");
    }

    /**
     * Moves the robot to the gate area.
     * @return The AutoBuilder instance for chaining.
     */
    public AutoBuilder gate() {
        autoMoves.add(AutoMove.GATE);
        return this;
    }

    public SequentialCommandGroup build() {
        SequentialCommandGroup commands = new SequentialCommandGroup();
        for (AutoMove move : autoMoves) {
            switch (move) {
                case PRELOAD:
                    break;
                case SPIKE_1:
                    break;
                case SPIKE_2:
                    break;
                case SPIKE_3:
                    break;
                case PARK_NEAR:
                    break;
                case PARK_FAR:
                    break;
                case GATE:
                    commands.addCommands(intakeGateCommand(), shootGateCommand());
                    break;
            }
        }
        commands.addCommands(new InstantCommand(this::setFinished));
        return commands;
    }
}
