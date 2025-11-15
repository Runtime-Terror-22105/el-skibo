package org.firstinspires.ftc.teamcode.robot.command;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.command.spindexer.SetSpindexerYawCommand;
import org.firstinspires.ftc.teamcode.robot.init.Robot;

public class TransferCommand extends SequentialCommandGroup {
    public TransferCommand(Robot robot) {
        super(
                new SetSpindexerTransferEnabledCommand(robot.spindexer, true),
                new WaitCommand(250),   // wait for ramp to go down
                new SetSpindexerYawCommand(robot.spindexer, robot.spindexer.getTargetYaw() + 2 * Math.PI),
                new WaitCommand(1000), // TODO: replace w/ end condition based off of spindexer encoder
                new SetSpindexerTransferEnabledCommand(robot.spindexer, false)
        );
    }
}
