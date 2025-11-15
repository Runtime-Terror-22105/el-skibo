package org.firstinspires.ftc.teamcode.robot.command.shooter;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;

public class ShootThreeBallsCommand extends InstantCommand {
    public static double spindexLoadingPower=0.5; // speed we set the SPINDEXER to spin and load into the shooter

    public SpindexerSubsystem spindexer;

    public ShooterSubsystem shooter;




    public ShootThreeBallsCommand(ShooterSubsystem shooter, SpindexerSubsystem spindexer) {
        // TODO: start auto-aiming once ShooterSubsystem supports it
        // throw new UnsupportedOperationException();
        this.shooter=shooter;
        this.spindexer=spindexer;

    }

    public void execute(){
        spindexer.setSpindexerPower(spindexLoadingPower);
    }





}