package org.firstinspires.ftc.teamcode.robot.command.states;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;
import org.firstinspires.ftc.teamcode.robot.command.shooter.SetShooterManualAimCommand;

public class GoToRestingStateCommand extends SequentialCommandGroup {
    public GoToRestingStateCommand(Robot robot) {
        super(new ParallelCommandGroup(
                new InstantCommand(() -> robot.robotState = RobotState.RESTING),
                new SetShooterManualAimCommand(robot.shooter, 0.0, 45.0, 0.0),
                new InstantCommand(() -> robot.intake.putDown())
                //TODO: spindexer has funnel out

        ));
    }
}
