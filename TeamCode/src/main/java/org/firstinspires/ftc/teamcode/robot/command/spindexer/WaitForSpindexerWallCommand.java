package org.firstinspires.ftc.teamcode.robot.command.spindexer;

import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;

public class WaitForSpindexerWallCommand extends WaitUntilCommand {
    public WaitForSpindexerWallCommand(SpindexerSubsystem spindexer) {
        super(spindexer::isWallDown);
    }
}
