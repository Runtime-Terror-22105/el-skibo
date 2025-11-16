package org.firstinspires.ftc.teamcode.robot.command.intake;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem;

public class SetIntakeDownCommand extends InstantCommand {
    // down = true -> put down
    // down = false -> put up
    public SetIntakeDownCommand(IntakeSubsystem intake, boolean down) {
        super(() -> {
            if (down) {
                intake.putDown();
            } else {
                intake.putUp();
            }
        });
    }
}
