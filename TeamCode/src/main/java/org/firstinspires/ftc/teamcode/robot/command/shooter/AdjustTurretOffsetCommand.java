package org.firstinspires.ftc.teamcode.robot.command.shooter;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.init.Robot;

public class AdjustTurretOffsetCommand extends InstantCommand {
    public static double INCREASE_AMOUNT_POS = 0.01;

    public AdjustTurretOffsetCommand(Robot robot, boolean increase) {
        super(() -> {
                    if (increase) robot.shooter.addTurretOffset(INCREASE_AMOUNT_POS);
                    else robot.shooter.addTurretOffset(-INCREASE_AMOUNT_POS);

                }
        );
    }
}
