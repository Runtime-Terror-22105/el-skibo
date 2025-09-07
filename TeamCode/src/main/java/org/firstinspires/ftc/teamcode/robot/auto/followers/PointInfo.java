package org.firstinspires.ftc.teamcode.robot.auto.followers;

import org.firstinspires.ftc.teamcode.math.Pose2d;

public class PointInfo {
    private final Pose2d goalPoint;
    private final Pose2d tolerances;
    private final double reachedTime;
    private final double speed;

    public PointInfo(Pose2d goalPoint, Pose2d tolerances, double speed, double reachedTime) {
        this.goalPoint = goalPoint;
        this.tolerances = tolerances;
        this.speed = speed;
        this.reachedTime = reachedTime;
    }

    public Pose2d getGoalPoint() {
        return this.goalPoint;
    }

    public Pose2d getTolerances() {
        return this.tolerances;
    }

    public double getSpeed() {
        return this.speed;
    }

    public double getReachedTime() {
        return this.reachedTime;
    }
}
