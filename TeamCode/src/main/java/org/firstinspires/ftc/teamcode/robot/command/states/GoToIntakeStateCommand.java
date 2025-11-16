package org.firstinspires.ftc.teamcode.robot.command.states;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.robot.command.intake.SetIntakeDownCommand;
import org.firstinspires.ftc.teamcode.robot.command.intake.SetIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.TransferCommand;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;
import org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem;

public class GoToIntakeStateCommand extends SequentialCommandGroup {
    public GoToIntakeStateCommand(Robot robot, TransferCommand transfer) {
        super(new ParallelCommandGroup(
                new InstantCommand(() -> robot.robotState = RobotState.INTAKING),
                new SetIntakeDownCommand(robot.intake, true),
                new SetIntakeSpeedCommand(robot.intake, IntakeSubsystem.defaultSpeed)
//              I don't think this should be here:  new InstantCommand(() -> transfer.setupTransfer())

        ));
    }
}
