package org.firstinspires.ftc.teamcode.robot.command.spindexer;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;

public class SetSpindexerWallDown extends InstantCommand {
    public SetSpindexerWallDown(SpindexerSubsystem spindexer, boolean down) {
        super(() -> {
            if (down) {
                spindexer.setWallDown();
            } else {
                spindexer.setWallUp();
            }
        });
    }
}
