package org.firstinspires.ftc.teamcode.robot.command.spindexer;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;

public class SetSpindexPidEnabledCommand extends InstantCommand {
    public SetSpindexPidEnabledCommand(SpindexerSubsystem spindexer, boolean enabled) {
        super(() -> spindexer.setPidEnabled(enabled));
    }
}
