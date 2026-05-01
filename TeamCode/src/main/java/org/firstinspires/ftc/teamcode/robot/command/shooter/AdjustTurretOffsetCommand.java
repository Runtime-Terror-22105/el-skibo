package org.firstinspires.ftc.teamcode.robot.command.shooter;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.init.Robot;

public class AdjustTurretOffsetCommand extends InstantCommand {
    public static double INCREASE_AMOUNT_DEG = 0.2;

    public AdjustTurretOffsetCommand(Robot robot, boolean increase) {
        super(() -> {
                    if (increase) robot.shooter.addTurretOffset(Math.toRadians(INCREASE_AMOUNT_DEG));
                    else robot.shooter.addTurretOffset(Math.toRadians(-INCREASE_AMOUNT_DEG));

                }
        );
    }
}
