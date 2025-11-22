package org.firstinspires.ftc.teamcode.robot.command.intake;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.subsystems.intake.IntakePitch;
import org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem;

public class SetIntakePitchCommand extends InstantCommand {
    public SetIntakePitchCommand(IntakeSubsystem intake, IntakePitch pitch) {
        super(() -> intake.setPitch(pitch));
    }
}
