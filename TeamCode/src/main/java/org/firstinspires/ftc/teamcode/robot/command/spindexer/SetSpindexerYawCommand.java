package org.firstinspires.ftc.teamcode.robot.command.spindexer;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;

// NB: this does NOT wait for the spindexer to reach the desired yaw; add a
// WaitForSpindexerYawCommand for that.
public class SetSpindexerYawCommand extends InstantCommand {
    public SetSpindexerYawCommand(SpindexerSubsystem spindexer, double yaw) {
        super(() -> spindexer.goToAngle120(yaw));
    }
    //sus cause we doen actually check the bool - watch out!
    public SetSpindexerYawCommand(SpindexerSubsystem spindexer, double yaw, boolean use120) {
        super(() -> {
            if (use120) {
                spindexer.goToAngle120(yaw);
            } else {
                spindexer.goToAngle360(yaw);
            }
        });

    }
}
