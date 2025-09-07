package org.firstinspires.ftc.teamcode.robot.drive.localizer;

import org.firstinspires.ftc.teamcode.math.Pose2d;

public abstract class Localizer {
    public abstract Pose2d getPosition();
    public abstract Pose2d getVelocity();

    public boolean isPinpointCooked() {
        return false;
    }
}
