package org.firstinspires.ftc.teamcode.robot.command.spindexer;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;

public class SetSpindexerYawCommand extends InstantCommand {
    public SetSpindexerYawCommand(SpindexerSubsystem spindexer, double yaw) {
        super(() -> spindexer.setYaw(yaw));
    }
}
