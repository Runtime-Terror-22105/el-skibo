package org.firstinspires.ftc.teamcode.robot.command.states;

import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;

public class ChangeHangStateCommand extends ConditionalCommand {
    public ChangeHangStateCommand(Robot robot) {
        super(
                new InstantCommand(() -> robot.robotState = RobotState.HANGING_FINAL),
                new InstantCommand(() -> robot.robotState = RobotState.HANGING_90),
                () -> robot.robotState == RobotState.HANGING_90
        );
    }
}
