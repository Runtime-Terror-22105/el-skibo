package org.firstinspires.ftc.teamcode.robot.command.spindexer;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.LogCatCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.robot.command.intake.SetIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.robot.command.shooter.SetShooterRPMCommand;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;
import org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;

@Config
public class PrepareShootCommand extends SequentialCommandGroup {
    public static long TIME_BEFORE_REVERSE_INTAKE = 150;
    public static long REVERSE_INTAKE_TIME_MS = 150;
    public static int RAMP_DELAY = 0;  // milliseconds
    public static long DELAY_BEFORE_CHANGING_SPINDEXER_YAW = 100;
    public static long SPINDEXER_TIMEOUT = 600L; //600 if not sorting, longer if we are
    //someone code that pls


    public PrepareShootCommand(Robot robot) {
        this(robot, null, null);
    }

    public PrepareShootCommand(Robot robot, Double hoodAngle, Double rpm) {
        super(
                new InstantCommand(() -> robot.robotState = RobotState.TRANSFER),
                new LogCatCommand("PrepareShootCommand", "Beginning prepare shoot", Log.INFO),

                // Phase 1: ???
                new InstantCommand(() -> robot.shooter.isAutoVelOn = rpm == null),
                new InstantCommand(() -> {
                    robot.shooter.isAutoHoodOn = hoodAngle == null;
                    if(hoodAngle != null)
                    {
                        robot.shooter.goalPitch = hoodAngle;
                    }
                }),
                new WaitCommand(TIME_BEFORE_REVERSE_INTAKE),
                new SetIntakeSpeedCommand(robot.intake, IntakeSubsystem.REVERSE_SPEED),
                new WaitCommand(REVERSE_INTAKE_TIME_MS),
                new ParallelCommandGroup(
                    new SetIntakeSpeedCommand(robot.intake, IntakeSubsystem.DEFAULT_SPEED),
                    new SetSpindexerWallDown(robot.spindexer, false),
                    new SetShooterRPMCommand(robot.shooter, rpm)
                ),
                new WaitCommand(DELAY_BEFORE_CHANGING_SPINDEXER_YAW), // todo: adjust this delay based on how long it takes for these two servos
                new LogCatCommand("PrepareShootCommand", "Phase 1 done", Log.INFO),

                // Phase 2/3: Sort the balls, spin to pre-transfer yaw
                new ConditionalCommand(
                        new SortCommand(robot),
                        new SetSpindexerYawCommand(robot.spindexer, SpindexerSubsystem.READY_POSITION),
                        robot::getAutoSort
                ),
                new WaitForSpindexerYawCommand(robot.spindexer).withTimeout(SPINDEXER_TIMEOUT),
                new LogCatCommand("PrepareShootCommand", "Phase 2/3 done", Log.INFO),

                // Phase 4: drop down ramp and start intake
                new SetSpindexerRampActive(robot.spindexer, true),
                new WaitCommand(RAMP_DELAY), // todo: adjust this delay based on how long it takes for ramp to drop
                new LogCatCommand("PrepareShootCommand", "Phase 4 done", Log.INFO),
                new SetIntakeSpeedCommand(robot.intake, 0),
                new InstantCommand(() -> robot.robotState = RobotState.READY_TO_SHOOT)
        );
    }
}
