package org.firstinspires.ftc.teamcode.robot.command.spindexer;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;

@Config
public class AdjustSpindexZeroCommand extends InstantCommand {
    public static double INCREASE_AMOUNT_DEGREES = 1;

    @Deprecated
    //Only works if using motor encoder, not absolute
    public AdjustSpindexZeroCommand(Robot robot, boolean increase) {
        super(() -> {
            Log.d("spindexer", "Changing offset, direction: "+ increase);
                    robot.spindexer.setHomedSpindexerOffset(
                            robot.spindexer.getHomedSpindexerOffset() +
                                    Math.toRadians(INCREASE_AMOUNT_DEGREES) * (increase ? -1 : 1));

                    robot.spindexer.goToAngle120(0);
                }
        );
    }
}