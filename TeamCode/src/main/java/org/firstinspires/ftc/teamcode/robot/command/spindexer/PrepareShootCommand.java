package org.firstinspires.ftc.teamcode.robot.command.spindexer;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.LogCatCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.command.intake.SetIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;
import org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;

@Config
public class PrepareShootCommand extends SequentialCommandGroup {
    public static long TIME_BEFORE_REVERSE_INTAKE = 300;
    public static long REVERSE_INTAKE_TIME_MS = 300;
    public static int RAMP_DELAY = 0;  // milliseconds
    public static long DELAY_BEFORE_CHANGING_SPINDEXER_YAW_IF_SORTING = 100;

    public PrepareShootCommand(Robot robot) {
        this(robot, false);
    }

    public PrepareShootCommand(Robot robot, boolean doReverseIntake) {
        super(
                new InstantCommand(() -> robot.robotState = RobotState.TRANSFER),
                new LogCatCommand("PrepareShootCommand", "Beginning prepare shoot", Log.INFO),

                // Phase 1: ???
                new InstantCommand(() -> robot.shooter.isAutoVelOn = true),
                new InstantCommand(() -> robot.shooter.isAutoHoodOn = true),
                new SetSpindexerWallDown(robot.spindexer, false),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                            new WaitCommand(TIME_BEFORE_REVERSE_INTAKE),
                            new SetIntakeSpeedCommand(robot.intake, IntakeSubsystem.REVERSE_SPEED),
                            new WaitCommand(REVERSE_INTAKE_TIME_MS)
                        ),
                        new InstantCommand(() -> {}),
                        () -> doReverseIntake
                ),
                new SetIntakeSpeedCommand(robot.intake, IntakeSubsystem.DEFAULT_SPEED),
                new LogCatCommand("PrepareShootCommand", "Phase 1 done", Log.INFO),

                // Phase 2/3: Sort the balls, spin to pre-transfer yaw
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new WaitCommand(DELAY_BEFORE_CHANGING_SPINDEXER_YAW_IF_SORTING), // todo: adjust this delay based on how long it takes for these two servos
                                new SortCommand(robot),
                                new SetSpindexerRampActive(robot.spindexer, true),
                                new InstantCommand(() -> robot.spindexer.useMaxPower = true)
                        ),
                        new ParallelCommandGroup(
                                new SetSpindexerYawCommand(robot.spindexer, SpindexerSubsystem.READY_POSITION),
                                new SetSpindexerRampActive(robot.spindexer, true)),
                        robot::getAutoSort
                ),
                new LogCatCommand("PrepareShootCommand", "Phase 2/3 done", Log.INFO),

                // Phase 4: drop down ramp and start intake

                new WaitCommand(RAMP_DELAY), // todo: adjust this delay based on how long it takes for ramp to drop
                new LogCatCommand("PrepareShootCommand", "Phase 4 done", Log.INFO),
                new SetIntakeSpeedCommand(robot.intake, 0),
                new InstantCommand(() -> robot.robotState = RobotState.READY_TO_SHOOT)

        );
    }
}
