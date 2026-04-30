package org.firstinspires.ftc.teamcode.robot.command;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.subsystems.ShooterSubsystem;

public class toggleAccelCompCommand extends InstantCommand {

    public toggleAccelCompCommand (ShooterSubsystem shooter){

        super(() -> {
            if (shooter.sotmAccelOverride == null){
                shooter.sotmAccelOverride = true;
            }
            else {
                shooter.sotmAccelOverride = !shooter.sotmAccelOverride;
            }
        });
    }

}
