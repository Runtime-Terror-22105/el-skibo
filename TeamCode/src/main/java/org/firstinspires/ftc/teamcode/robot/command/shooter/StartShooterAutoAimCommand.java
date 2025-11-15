package org.firstinspires.ftc.teamcode.robot.command.shooter;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.subsystems.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.ShooterSubsystem;

public class StartShooterAutoAimCommand extends InstantCommand {

    public ShooterSubsystem shooter;
    public LocalizationSubsystem localizer;

    public Pose2d goal;

    public StartShooterAutoAimCommand(ShooterSubsystem shooter, LocalizationSubsystem localizer, Pose2d goal) {
        // TODO: start auto-aiming once ShooterSubsystem supports it
        this.shooter=shooter;
        this.localizer=localizer;
        this.goal=goal;
    }

    public void aim(){
        shooter.doAutoShoot(localizer.getCurrentPosition(),goal);
    }


}
