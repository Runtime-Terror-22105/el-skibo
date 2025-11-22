package org.firstinspires.ftc.teamcode.robot.command.spindexer;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;

// NB: this does NOT wait for the spindexer to reach the desired yaw; add a
// WaitForSpindexerYawCommand for that.
public class ChangeSpindexerYawCommand extends InstantCommand {
    public ChangeSpindexerYawCommand(SpindexerSubsystem spindexer, double yaw) {
        super(() -> spindexer.setYaw(spindexer.getTargetYaw() + yaw));
    }
}
