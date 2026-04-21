package org.firstinspires.ftc.teamcode.robot.command.intake;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem;

public class DropIntakeCommand extends InstantCommand {
    public DropIntakeCommand(IntakeSubsystem intake, boolean isDown) {
        super(() -> intake.setIntakeDropped(isDown));
    }
}
