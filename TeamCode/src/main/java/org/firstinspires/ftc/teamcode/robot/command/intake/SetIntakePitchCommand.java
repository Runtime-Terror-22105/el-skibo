package org.firstinspires.ftc.teamcode.robot.command.intake;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem;

public class SetIntakePitchCommand extends InstantCommand {
    public SetIntakePitchCommand(IntakeSubsystem intake, double pitch) {
        super(() -> intake.setPitch(pitch));
    }
}
