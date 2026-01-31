package org.firstinspires.ftc.teamcode.robot.auto;

import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.init.Robot;

import java.util.ArrayList;
import java.util.List;

import kotlin.NotImplementedError;

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

    private final Follower follower;
    private final Pose2d startPose;
    private final List<AutoMove> autoMoves;

    public AutoBuilder(Robot robot, Team team, StartingConfiguration initial) {
        this.follower = robot.follower;
        this.startPose = getStartPose(team, initial);

        this.follower.setStartingPose(startPose.toPedro());
        robot.goalPos = team.getGoalPos();
        autoMoves = new ArrayList<>();
    }

    private static Pose2d getStartPose(Team team, StartingConfiguration initial) {
        if (initial == StartingConfiguration.NEAR) {
            return team.getStartPosNear();
        } else {
            return team.getStartPosFar();
        }
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
    public AutoBuilder parkFar() {
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
                    break;
            }
        }
        return new SequentialCommandGroup();
    }
}
