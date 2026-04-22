package org.firstinspires.ftc.teamcode.robot.command.shooter;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.init.Robot;

public class ToggleAutoTurretCommand extends InstantCommand {

    public ToggleAutoTurretCommand(Robot robot, boolean on, double newPosRad){
        super( () -> {
            if (!on){
                robot.shooter.isAutoTurretOn = false;
                robot.shooter.setTurretAngle(newPosRad);
            }
            else robot.shooter.isAutoTurretOn = true;
        });
    }

    public ToggleAutoTurretCommand(Robot robot, boolean on){
        super( () -> {
            if (!on){
                robot.shooter.isAutoTurretOn = false;
            }
            else robot.shooter.isAutoTurretOn = true;
        });

    }
}
