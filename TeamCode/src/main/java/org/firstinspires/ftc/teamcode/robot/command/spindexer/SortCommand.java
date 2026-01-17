package org.firstinspires.ftc.teamcode.robot.command.spindexer;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.robot.command.intake.SetIntakePitchCommand;
import org.firstinspires.ftc.teamcode.robot.command.intake.SetIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;
import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake.IntakePitch;

@Config
public class SortCommand extends SequentialCommandGroup {

    public static long alignTimeMS = 100;
    public static long WAiT_TiMEOUT_MOTiF = 1500;

    public SortCommand(Robot robot){
        super(
                new InstantCommand(()->robot.spindexer.goToNearestSide()),
                new WaitForSpindexerYawCommand(robot.spindexer).withTimeout(alignTimeMS),
                new WaitUntilCommand(() -> robot.camera.getGlyph() != null).withTimeout(WAiT_TiMEOUT_MOTiF),
                new InstantCommand(()->robot.spindexer.newSort())
        );
    }


}
