package org.firstinspires.ftc.teamcode.robot.command.spindexer;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;

public class SetSpindexerPoleActive extends InstantCommand {
    public SetSpindexerPoleActive(SpindexerSubsystem spindexer, boolean active) {
        super(() -> {
            if (active) {
                spindexer.activatePole();
            } else {
                spindexer.deactivatePole();
            }
        });
    }
}
