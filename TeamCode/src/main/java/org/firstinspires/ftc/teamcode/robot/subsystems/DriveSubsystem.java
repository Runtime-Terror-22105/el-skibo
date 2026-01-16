package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.util.Profiler;

import com.skeletonarmy.marrow.zones.PolygonZone;
import com.skeletonarmy.marrow.zones.Point;

public class DriveSubsystem extends SubsystemBase {
    private final Robot robot;

    public DriveSubsystem(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void periodic() {
        try (Profiler.Scope p = Profiler.enter("DriveSubsystem")) {
            robot.follower.update();
            robot.robotZone.setPosition(robot.follower.getPose().getX(), robot.follower.getPose().getY());
            robot.robotZone.setRotation(robot.follower.getHeading());
        }
    }
}
