package org.firstinspires.ftc.teamcode.robot.command.shooter;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.subsystems.ShooterSubsystem;

public class SetShooterRPMCommand extends InstantCommand {
    public SetShooterRPMCommand(ShooterSubsystem shooter, double rpm) {
        super(() -> shooter.setSpeed(rpm));
    }
}
