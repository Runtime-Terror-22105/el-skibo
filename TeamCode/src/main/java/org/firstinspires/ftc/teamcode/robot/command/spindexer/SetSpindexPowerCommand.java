package org.firstinspires.ftc.teamcode.robot.command.spindexer;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;

public class SetSpindexPowerCommand extends InstantCommand {
    /**
     * Note: You must first disable the spindexer PID before using this command, otherwise the PID will just set the power to the current target.
     * @param spindexer
     * @param power
     */
    public SetSpindexPowerCommand(SpindexerSubsystem spindexer, double power) {
        super(() -> spindexer.setSpindexerPower(power));
    }

}
