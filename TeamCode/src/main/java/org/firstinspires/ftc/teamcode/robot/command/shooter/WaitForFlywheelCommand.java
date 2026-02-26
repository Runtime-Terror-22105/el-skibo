package org.firstinspires.ftc.teamcode.robot.command.shooter;

import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.robot.subsystems.ShooterSubsystem;

public class WaitForFlywheelCommand extends WaitUntilCommand {
    public WaitForFlywheelCommand(ShooterSubsystem shooter) {
        super(shooter::isFlywheelAtTarget);
    }
}
