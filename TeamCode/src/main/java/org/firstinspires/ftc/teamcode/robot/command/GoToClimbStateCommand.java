package org.firstinspires.ftc.teamcode.robot.command;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;

public class GoToClimbStateCommand extends SequentialCommandGroup {
    public GoToClimbStateCommand(Robot robot) {
        super(new ParallelCommandGroup(
                new InstantCommand(() -> robot.robotState = RobotState.CLIMBING),
                //TODO: activate pto
                new InstantCommand(() -> {
                    robot.hardware.motorRearLeft.setPower(1.0);
                    robot.hardware.motorRearRight.setPower(1.0);
                })
        ));
    }
}
