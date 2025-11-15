package org.firstinspires.ftc.teamcode.robot.command.states;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.robot.command.spindexer.TransferCommand;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;
import org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem;

public class GoToIntakeStateCommand extends SequentialCommandGroup {
    public GoToIntakeStateCommand(Robot robot, TransferCommand transfer) {
        super(new ParallelCommandGroup(
                new InstantCommand(() -> robot.robotState = RobotState.INTAKING),
                new InstantCommand(() -> robot.intake.putUp()),
                new InstantCommand(() -> robot.intake.setSpeed(IntakeSubsystem.defaultSpeed)),
                new InstantCommand(() -> transfer.setupTransfer())

        ));
    }
}
