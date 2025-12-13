package org.firstinspires.ftc.teamcode.robot.command.shooter;

import com.acmerobotics.dashboard.config.Config;
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

@Config
public class ShootThreeBallsCommand extends SequentialCommandGroup {
//    public static double SPINDEX_ROTATIONS = -4.5;  // revolutions, negative bc clockwise
    public static double SPINDEX_TRANSFER_POWER = -1;
    public static int SPINDEX_TRANSFER_TIME = 2000;  // milliseconds

    private final Robot robot;

    public ShootThreeBallsCommand(Robot robot) {
        super(
                new InstantCommand(() -> robot.robotState = RobotState.SHOOTING),
                new SetIntakeSpeedCommand(robot.intake, IntakeSubsystem.DEFAULT_SPEED),
                new WaitCommand(200),

                // Phase 5: transfer balls
                new InstantCommand(() -> {
                    robot.spindexer.setPidEnabled(false);
                    robot.spindexer.setSpindexerPower(SPINDEX_TRANSFER_POWER);
                }),
                new WaitCommand(SPINDEX_TRANSFER_TIME),
                new InstantCommand(() -> {
                    robot.spindexer.setSpindexerPower(0.0);
                    robot.spindexer.setPidEnabled(true);
                }),

                // reset spindexer, intake, shooter, and pole
                new ParallelCommandGroup(
                        new SetIntakeSpeedCommand(robot.intake, 0),
                        new SetSpindexerPoleActive(robot.spindexer, false),
                        new SetSpindexerRampActive(robot.spindexer, false),
                        new SetSpindexerYawCommand(robot.spindexer, 0.0)
                ),
                new InstantCommand(() -> robot.robotState = RobotState.RESTING)
        );
        this.robot = robot;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        robot.spindexer.setPidEnabled(true);
    }
}