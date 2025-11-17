package org.firstinspires.ftc.teamcode.robot.command.spindexer;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;

public class SetSpindexerRampActive extends InstantCommand {
    public SetSpindexerRampActive(SpindexerSubsystem spindexer, boolean active) {
        super(() -> {
            if (active) {
                spindexer.enableRamp();
            } else {
                spindexer.disableRamp();
            }
        });
    }
}
