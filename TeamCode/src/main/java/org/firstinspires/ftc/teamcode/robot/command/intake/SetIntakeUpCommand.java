package org.firstinspires.ftc.teamcode.robot.command.intake;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem;

public class SetIntakeUpCommand extends InstantCommand {
    public SetIntakeUpCommand(IntakeSubsystem intake, boolean isUp) {
        super(() -> intake.setIntakeLifted(isUp));
    }
}
