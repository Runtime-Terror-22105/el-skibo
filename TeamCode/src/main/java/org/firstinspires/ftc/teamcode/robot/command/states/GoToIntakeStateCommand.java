package org.firstinspires.ftc.teamcode.robot.command.states;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.LogCatCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.robot.command.intake.IntakeLiftedCommand;
import org.firstinspires.ftc.teamcode.robot.command.intake.SetIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.SetSpindexerWallDown;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.SetSpindexerYawCommand;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;
import org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem;

public class GoToIntakeStateCommand extends SequentialCommandGroup {
    public GoToIntakeStateCommand(Robot robot) {
        super(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.robotState = RobotState.INTAKING),
                        new LogCatCommand("intake testing", "just set state"),
                        new SetIntakeSpeedCommand(robot.intake, IntakeSubsystem.DEFAULT_SPEED),
                        new IntakeLiftedCommand(robot.intake,false),
                        new SetSpindexerYawCommand(robot.spindexer, 0),
                        new LogCatCommand("intake testing", "set spindexer")
                ),
                new SetSpindexerWallDown(robot.spindexer, true)
        );
    }
}
