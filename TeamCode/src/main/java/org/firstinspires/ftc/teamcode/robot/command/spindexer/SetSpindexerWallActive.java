package org.firstinspires.ftc.teamcode.robot.command.spindexer;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;

public class SetSpindexerWallActive extends InstantCommand {
    public SetSpindexerWallActive(SpindexerSubsystem spindexer, boolean active) {
        super(() -> {
            if (active) {
                spindexer.setWallActive();
            } else {
                spindexer.setWallDeactive();
            }
        });
    }
}
