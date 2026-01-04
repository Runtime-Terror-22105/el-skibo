package org.firstinspires.ftc.teamcode.robot.command.spindexer;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.command.intake.SetIntakePitchCommand;
import org.firstinspires.ftc.teamcode.robot.command.intake.SetIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;
import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake.IntakePitch;

public class SortCommand extends SequentialCommandGroup {

    public static long alignTimeMS = 100;
    public SortCommand(Robot robot){
        super(
                new InstantCommand(()->robot.spindexer.goToNearestSide()),
                new WaitForSpindexerYawCommand(robot.spindexer).withTimeout(alignTimeMS),
                new InstantCommand(()->robot.spindexer.sortBalls())
        );
    }


}
