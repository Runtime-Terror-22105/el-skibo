package org.firstinspires.ftc.teamcode.robot.command.states;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;

public class GoToHang90StateCommand extends SequentialCommandGroup {
    public GoToHang90StateCommand(Robot robot) {
        super(
                new InstantCommand(() -> robot.robotState = RobotState.HANGING_90),
                new InstantCommand(() -> robot.hang.goTo90DegreePosition())
        )
    }
}
