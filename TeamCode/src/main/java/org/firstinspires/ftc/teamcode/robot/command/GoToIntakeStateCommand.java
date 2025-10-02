package org.firstinspires.ftc.teamcode.robot.command;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;
import org.firstinspires.ftc.teamcode.robot.subsystems.shooter.SetShooterManualAimCommand;

public class GoToIntakeStateCommand extends SequentialCommandGroup {
    public GoToIntakeStateCommand(Robot robot) {
        super(new ParallelCommandGroup(
                new InstantCommand(() -> robot.robotState = RobotState.INTAKING)
                //TODO: if intake is up, put down
                //TODO: spin intake in
                //TODO: spindexer has funnel out
        ));
    }
}
