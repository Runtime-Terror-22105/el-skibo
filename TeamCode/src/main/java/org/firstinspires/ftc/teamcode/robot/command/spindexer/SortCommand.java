package org.firstinspires.ftc.teamcode.robot.command.spindexer;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;

public class SortCommand extends InstantCommand {
    public SortCommand (SpindexerSubsystem spindexer){
        super(() -> spindexer.sortBalls());
    }
}
