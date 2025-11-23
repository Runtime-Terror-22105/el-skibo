package org.firstinspires.ftc.teamcode.robot.command.spindexer;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;

public class AdjustSpindexZeroCommand extends InstantCommand {
    public static double INCREASE_AMOUNT_DEGREES = 5;

    public AdjustSpindexZeroCommand(Robot robot, boolean increase) {
        super(() -> {
                    robot.spindexer.setHomedSpindexerOffset(
                            robot.spindexer.getHomedSpindexerOffset() +
                                    SpindexerSubsystem.radiansToTicks(Math.toRadians(INCREASE_AMOUNT_DEGREES)) * (increase ? -1 : 1));

                    robot.spindexer.goToAngle120(0);
                }
        );
    }
}
