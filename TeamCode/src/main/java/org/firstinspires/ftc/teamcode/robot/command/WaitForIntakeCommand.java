package org.firstinspires.ftc.teamcode.robot.command;

import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.util.ArrayUtil;
import org.firstinspires.ftc.teamcode.util.BallColor;

// Waits until 3 balls are in the spindexer
public class WaitForIntakeCommand extends WaitUntilCommand {
    public WaitForIntakeCommand(Robot robot) {
        super(() -> !ArrayUtil.contains(robot.spindexer.getBallPositions(), BallColor.NONE));
    }
}
