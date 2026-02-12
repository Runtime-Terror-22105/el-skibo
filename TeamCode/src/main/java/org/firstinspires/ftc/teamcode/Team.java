package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.math.Pose2d;

import java.util.function.Supplier;

public enum Team {
    BLUE(
            () -> FieldConstants.BLUE_GOAL_POS,
            () -> FieldConstants.BLUE_START_POS_NEAR,
            () -> FieldConstants.BLUE_START_POS_FAR,
            () -> FieldConstants.BLUE_KEY
    ),
    RED(
            () -> FieldConstants.RED_GOAL_POS,
            () -> FieldConstants.RED_START_POS_NEAR,
            () -> FieldConstants.RED_START_POS_FAR,
            () -> FieldConstants.RED_KEY
    );

    private final Supplier<Pose2d> goalPos;
    private final Supplier<Pose2d> startPosNear;
    private final Supplier<Pose2d> startPosFar;
    private final Supplier<String> blackboardKey;

    Team(Supplier<Pose2d> goalPos, Supplier<Pose2d> startPosNear, Supplier<Pose2d> startPosFar, Supplier<String> blackboardKey) {
        this.goalPos = goalPos;
        this.startPosNear = startPosNear;
        this.startPosFar = startPosFar;
        this.blackboardKey = blackboardKey;
    }

    public Pose2d getGoalPos() {
        return goalPos.get();
    }

    public Pose2d getStartPosNear() {
        return startPosNear.get();
    }

    public Pose2d getStartPosFar() {
        return startPosFar.get();
    }

    public String getBlackboardKey(){return blackboardKey.get();}

}
