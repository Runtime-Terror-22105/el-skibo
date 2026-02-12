package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.util.StartConfig;

import java.util.function.Supplier;

public enum Team {
    BLUE(
            () -> FieldConstants.BLUE_GOAL_POS,
            () -> FieldConstants.BLUE_KEY
    ),
    RED(
            () -> FieldConstants.RED_GOAL_POS,
            () -> FieldConstants.RED_KEY
    );

    private final Supplier<Pose2d> goalPos;
    private final Supplier<String> blackboardKey;

    Team(Supplier<Pose2d> goalPos, Supplier<String> blackboardKey) {
        this.goalPos = goalPos;
        this.blackboardKey = blackboardKey;
    }

    public Pose2d getGoalPos() {
        return goalPos.get();
    }

    public String getBlackboardKey(){return blackboardKey.get();}

    public Pose2d getStartPose(StartConfig config) {
        Pose2d startPoseBlue = config.getStartPoseBlue();
        return RED.equals(this) ? startPoseBlue.mirror() : startPoseBlue;
    }

    @Deprecated
    public Pose2d getStartPosNear() {
        return getStartPose(StartConfig.NEAR);
    }

    @Deprecated
    public Pose2d getStartPosFar() {
        return getStartPose(StartConfig.FAR);
    }
}
