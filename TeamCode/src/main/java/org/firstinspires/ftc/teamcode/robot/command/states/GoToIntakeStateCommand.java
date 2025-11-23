package org.firstinspires.ftc.teamcode.robot.command.states;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.robot.command.intake.SetIntakePitchCommand;
import org.firstinspires.ftc.teamcode.robot.command.intake.SetIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.SetSpindexerWallDown;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake.IntakePitch;
import org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem;

public class GoToIntakeStateCommand extends SequentialCommandGroup {
    public GoToIntakeStateCommand(Robot robot) {
        super(new ParallelCommandGroup(
                new InstantCommand(() -> robot.robotState = RobotState.INTAKING),
                new SetIntakePitchCommand(robot.intake, IntakePitch.DOWN),
                new SetIntakeSpeedCommand(robot.intake, IntakeSubsystem.DEFAULT_SPEED),
                new SetSpindexerWallDown(robot.spindexer, true)
//              I don't think this should be here:  new InstantCommand(() -> transfer.setupTransfer())

        ));
    }
}
