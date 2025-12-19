package org.firstinspires.ftc.teamcode.robot.command.states;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;
import org.firstinspires.ftc.teamcode.robot.subsystems.HangSubsystem;

public class GoToClimbStateCommand extends SequentialCommandGroup {
    public Robot robot;
    public GoToClimbStateCommand(Robot robot, HangSubsystem.Position position) {
        super(new ParallelCommandGroup(
                // note: setting climb state disables spindexer pto in SpindexerSubsystem
                new InstantCommand(() -> robot.robotState = RobotState.CLIMBING),
                new InstantCommand(() -> robot.hang.setPTOState(true)),
                new InstantCommand(() -> robot.hang.setPosition(position))
        ));
    }
}
