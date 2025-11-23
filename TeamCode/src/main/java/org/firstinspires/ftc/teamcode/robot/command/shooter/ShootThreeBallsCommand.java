package org.firstinspires.ftc.teamcode.robot.command.shooter;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.command.intake.SetIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.ChangeSpindexerYawCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.SetSpindexerPoleActive;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.SetSpindexerRampActive;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.SetSpindexerYawCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.WaitForSpindexerYawCommand;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;
import org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem;

public class ShootThreeBallsCommand extends SequentialCommandGroup {
    public static double SPINDEX_ROTATIONS = -4.5;  // revolutions, negative bc clockwise

    public ShootThreeBallsCommand(Robot robot) {
        super(
                new InstantCommand(() -> robot.robotState = RobotState.SHOOTING),
                new SetIntakeSpeedCommand(robot.intake, IntakeSubsystem.DEFAULT_SPEED),
                new WaitCommand(200),

                // Phase 5: transfer balls
                new ChangeSpindexerYawCommand(robot.spindexer, SPINDEX_ROTATIONS*2*Math.PI),
                new WaitForSpindexerYawCommand(robot.spindexer).withTimeout(2000),

                // reset spindexer, intake, shooter, and pole
                new ParallelCommandGroup(
                        new SetIntakeSpeedCommand(robot.intake, 0),
                        new SetSpindexerPoleActive(robot.spindexer, false),
                        new SetSpindexerRampActive(robot.spindexer, false),
                        new SetSpindexerYawCommand(robot.spindexer, 0.0)
                ),
                new InstantCommand(() -> robot.robotState = RobotState.RESTING)
        );
    }
}