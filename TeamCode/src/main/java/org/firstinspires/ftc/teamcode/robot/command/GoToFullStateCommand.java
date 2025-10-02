package org.firstinspires.ftc.teamcode.robot.command;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;

public class GoToFullStateCommand extends SequentialCommandGroup {
    public GoToFullStateCommand(Robot robot) {
        super(new ParallelCommandGroup(
                new InstantCommand(() -> robot.robotState = RobotState.FULL)
                //TODO: intake spin oppisite
                //TODO: start running autoaim function in loop
        ));
    }
}
