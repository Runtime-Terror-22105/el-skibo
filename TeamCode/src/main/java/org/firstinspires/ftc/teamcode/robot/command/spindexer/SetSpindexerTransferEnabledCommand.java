package org.firstinspires.ftc.teamcode.robot.command.spindexer;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;

public class SetSpindexerTransferEnabledCommand extends InstantCommand {
    public SetSpindexerTransferEnabledCommand(SpindexerSubsystem spindexer, boolean enabled) {
        super(() -> {
            if (enabled) {
                spindexer.activateTransfer();
            } else {
                spindexer.deactivateTransfer();
            }
        });
    }
}
