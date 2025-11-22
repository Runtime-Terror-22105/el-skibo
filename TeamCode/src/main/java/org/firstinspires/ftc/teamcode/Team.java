package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.math.Pose2d;

import java.util.function.Supplier;

public enum Team {
    BLUE(() -> FieldConstants.BLUE_GOAL_POS, () -> FieldConstants.BLUE_START_POS_AUTO),
    RED(() -> FieldConstants.RED_GOAL_POS, () -> FieldConstants.RED_START_POS_AUTO);

    private final Supplier<Pose2d> goalPos;
    private final Supplier<Pose2d> startPosAuto;

    Team(Supplier<Pose2d> goalPos, Supplier<Pose2d> startPosAuto) {
        this.goalPos = goalPos;
        this.startPosAuto = startPosAuto;
    }

    public Pose2d getGoalPos() {
        return goalPos.get();
    }

    public Pose2d getStartPosAuto() {
        return startPosAuto.get();
    }
}
