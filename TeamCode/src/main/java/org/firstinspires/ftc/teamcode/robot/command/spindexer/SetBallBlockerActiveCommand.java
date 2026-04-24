package org.firstinspires.ftc.teamcode.robot.command.spindexer;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;

public class SetBallBlockerActiveCommand extends InstantCommand {
    public SetBallBlockerActiveCommand(SpindexerSubsystem spindexer, boolean active){
        super(() -> {
            if (active) {
                spindexer.activateBlocker();
            } else {
                spindexer.deactivateBlocker();
            }
        });
    }

}
