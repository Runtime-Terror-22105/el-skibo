package org.firstinspires.ftc.teamcode.robot.command.shooter;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;

public class ShootThreeBallsCommand extends InstantCommand {
    SpindexerSubsystem spindexer;
    ShooterSubsystem shooter;
    public ShootThreeBallsCommand(ShooterSubsystem shooter, SpindexerSubsystem spindexer) {
        // TODO: start auto-aiming once ShooterSubsystem supports it
        // throw new UnsupportedOperationException();
        this.shooter=shooter;
        this.spindexer=spindexer;
    }


}