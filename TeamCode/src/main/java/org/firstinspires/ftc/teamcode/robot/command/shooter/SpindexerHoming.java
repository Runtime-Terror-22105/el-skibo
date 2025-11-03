package org.firstinspires.ftc.teamcode.robot.command.shooter;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;

public class SpindexerHoming extends InstantCommand {

    public void HomeSpindexer(SpindexerSubsystem spindexer){
        while(!spindexer.getLimitSwitchState()) {
            spindexer.setSpindexerPower(0.5);
        }
        spindexer.setSpindexerOffset(spindexer.getPosition());
        spindexer.setSpindexerPower(0.0);
    }
}
