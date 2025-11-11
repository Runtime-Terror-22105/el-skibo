package org.firstinspires.ftc.teamcode.robot.command.intake;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem;

public class SetIntakeSpeedCommand extends InstantCommand {
    public SetIntakeSpeedCommand(IntakeSubsystem intake, double speed) {
        super(() -> intake.setSpeed(speed));
    }
}
