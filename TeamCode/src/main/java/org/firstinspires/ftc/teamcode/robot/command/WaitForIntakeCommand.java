package org.firstinspires.ftc.teamcode.robot.command;

import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.robot.init.Robot;

// Waits until 3 balls are in the spindexer
public class WaitForIntakeCommand extends WaitUntilCommand {
    public WaitForIntakeCommand(Robot robot) {
        super(() -> {
            char[] balls = robot.spindexer.getBallPositions();
            return balls[0] != 'N' && balls[1] != 'N' && balls[2] != 'N';
        });
    }
}
