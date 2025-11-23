package org.firstinspires.ftc.teamcode.robot.command.states;

import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;

public class GoToClimbStateCommand extends SequentialCommandGroup {
    public Robot robot;
    public GoToClimbStateCommand(Robot robot) {
        super(new ParallelCommandGroup(
                new InstantCommand(() -> robot.robotState = RobotState.CLIMBING),
                new InstantCommand(() -> robot.hang.setPTOState(true))
        ));
    }
}
