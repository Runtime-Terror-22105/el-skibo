package org.firstinspires.ftc.teamcode.robot.command.spindexer;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.command.intake.SetIntakePitchCommand;
import org.firstinspires.ftc.teamcode.robot.command.intake.SetIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.robot.command.shooter.SetShooterRPMCommand;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake.IntakePitch;

@Config
public class PrepareShootCommand extends SequentialCommandGroup {
    public static double SHOOTER_RPM = 3500;
    public static double PRE_YAW_ANGLE = 30.0;  // degrees
    public static int PRE_YAW_DELAY = 250;  // milliseconds
    public static int RAMP_DELAY = 500;  // milliseconds
    public static long DELAY_BEFORE_CHANGING_SPINDEXER_YAW = 750;

    public PrepareShootCommand(Robot robot) {
        this(robot, null);
    }

    public PrepareShootCommand(Robot robot, Double rpm) {
        super(
                new InstantCommand(() -> robot.robotState = RobotState.READY_TO_SHOOT),

                // Phase 1 and 2: ???
                new InstantCommand(() -> robot.shooter.isAutoVelOn = rpm == null),
                new ParallelCommandGroup(
                    new SetIntakePitchCommand(robot.intake, IntakePitch.UP),
                    new SetIntakeSpeedCommand(robot.intake, 0),
                    new SetSpindexerWallDown(robot.spindexer, false),
                    new SetSpindexerPoleActive(robot.spindexer, true),
                    new SetShooterRPMCommand(robot.shooter, rpm)
                ),
                new WaitCommand(DELAY_BEFORE_CHANGING_SPINDEXER_YAW), // todo: adjust this delay based on how long it takes for these two servos

                // Phase 3: rotate to pre-transfer yaw
                // TODO: angle needs to be relative to current position, NOT absolute
                new SetSpindexerYawCommand(robot.spindexer, Math.toRadians(PRE_YAW_ANGLE)),
                new WaitForSpindexerYawCommand(robot.spindexer).withTimeout(2000),
                new WaitCommand(PRE_YAW_DELAY),

                // Phase 4: drop down ramp and start intake
                new SetSpindexerRampActive(robot.spindexer, true),
                new WaitCommand(RAMP_DELAY) // todo: adjust this delay based on how long it takes for ramp to drop
        );
    }
}
