package org.firstinspires.ftc.teamcode.robot.subsystems.shooter;

import com.seattlesolvers.solverslib.command.InstantCommand;

public class SetShooterManualAimCommand extends InstantCommand {
    public SetShooterManualAimCommand(ShooterSubsystem shooter, double velocity, double pitch, double yaw) {
        super(() -> shooter.manualAim(velocity, pitch, yaw), shooter);
    }
}
