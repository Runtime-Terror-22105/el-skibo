package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.math.Pose2d;

import java.util.function.Supplier;

public enum StartConfig {
    NEAR(() -> FieldConstants.BLUE_START_POS_NEAR),
    FAR(() -> FieldConstants.BLUE_START_POS_FAR),
    FIELD_CENTER(() -> FieldConstants.START_POS_CENTER),
    FAR_SORTED(() -> FieldConstants.BLUE_START_POS_FAR_SORTED),
    FAR_SIDE(() -> FieldConstants.BLUE_START_POS_FAR_SIDE); // only used for AutoFarVisionTest for convenince

    // Returns the start pose for BLUE team only.
    private final Supplier<Pose2d> startPoseBlue;

    StartConfig(Supplier<Pose2d> startPoseBlue) {
        this.startPoseBlue = startPoseBlue;
    }

    public Pose2d getStartPoseBlue() {
        return startPoseBlue.get();
    }
}
