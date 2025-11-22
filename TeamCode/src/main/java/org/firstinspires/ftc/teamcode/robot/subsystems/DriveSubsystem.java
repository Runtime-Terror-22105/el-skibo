package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.robot.init.Robot;

public class DriveSubsystem extends SubsystemBase {
    private final Robot robot;

    public DriveSubsystem(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void periodic() {
        robot.follower.update();
    }
}
