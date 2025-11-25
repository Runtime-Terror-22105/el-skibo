package org.firstinspires.ftc.teamcode.robot.command.shooter;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.subsystems.ShooterSubsystem;

public class StartShooterRejectCommand extends InstantCommand {
    ShooterSubsystem shooter;
    public StartShooterRejectCommand(ShooterSubsystem shooter) {
        this.shooter=shooter;

    }
    public void reject(){
        shooter.setGoalPitch(0.95);
        shooter.shooterPower =1.0;
        shooter.setSpeed(1.0);
    }
}
