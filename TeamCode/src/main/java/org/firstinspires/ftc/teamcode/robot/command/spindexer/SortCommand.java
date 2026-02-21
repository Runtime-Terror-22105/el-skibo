package org.firstinspires.ftc.teamcode.robot.command.spindexer;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.robot.init.Robot;

@Config
public class SortCommand extends SequentialCommandGroup {

    public static long alignTimeMS = 2000;
    public static long WAIT_TIMEOUT_MOTIF = 5000;
    public static long SPINDEXER_TIMEOUT = 600L;

    public SortCommand(Robot robot){
        super(

                new InstantCommand(()->robot.spindexer.goToNearestSide()),
                new WaitForSpindexerYawCommand(robot.spindexer).withTimeout(alignTimeMS),
                new WaitUntilCommand(() -> robot.camera.getGlyph() != null).withTimeout(WAIT_TIMEOUT_MOTIF),
                new InstantCommand(()->robot.spindexer.newSort()),
                new WaitForSpindexerYawCommand(robot.spindexer).withTimeout(SPINDEXER_TIMEOUT)

        );
    }
}
