package org.firstinspires.ftc.teamcode.robot.command.spindexer;

import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;

public class WaitForSpindexerYawCommand extends WaitUntilCommand {
    public WaitForSpindexerYawCommand(SpindexerSubsystem spindexer) {
        super(spindexer::atTargetYaw);
    }
}
