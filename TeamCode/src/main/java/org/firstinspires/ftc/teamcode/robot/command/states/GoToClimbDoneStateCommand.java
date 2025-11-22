package org.firstinspires.ftc.teamcode.robot.command.states;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;

public class GoToClimbDoneStateCommand extends SequentialCommandGroup {
    public GoToClimbDoneStateCommand(Robot robot) {
        super(new ParallelCommandGroup(
                new InstantCommand(() -> robot.robotState = RobotState.DONE_CLIMB),
                new InstantCommand(() -> {
//                    robot.hardware.motorRearLeft.setPower(0.7);
//                    robot.hardware.motorRearRight.setPower(0.7);
                })
        ));
    }
}
