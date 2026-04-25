package org.firstinspires.ftc.teamcode.robot.command.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.LogCatCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.command.intake.SetIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.SetSpindexPidEnabledCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.SetSpindexPowerCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.SetSpindexerRampActive;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.SetSpindexerYawCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToRestingStateCommand;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;
import org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;

@Config
public class ShootThreeBallsCommand extends SequentialCommandGroup {
    public static double ANGLE_THRESHOLD_SPEED_CHANGE = 210;
    //    public static double SPINDEX_ROTATIONS = -4.5;  // revolutions, negative bc clockwise
    public static double SPINDEX_TRANSFER_POWER = -1;
    public static int SPINDEX_TRANSFER_TIME = 500;// milliseconds
    public static double SPINDEX_SORTING_TRANSFER_POWER = 0.7;

    public static int reverseIntakeTimeMS = 150;
    public static int SPINDEX_SORTING_TRANSFER_TIME = 1366;//(int) (700/SpindexerSubsystem.MAX_POWER_SORTING);  // milliseconds

    private final Robot robot;

    public ShootThreeBallsCommand(Robot robot, double transferPower, boolean isTeleop) {
        super(
                new LogCatCommand("ShootThreeBallsCommand", "starting shoot"),
                new InstantCommand(() -> robot.robotState = RobotState.SHOOTING),
                new SetIntakeSpeedCommand(robot.intake, IntakeSubsystem.DEFAULT_SPEED),

                new SetSpindexPidEnabledCommand(robot.spindexer, false),
                new ConditionalCommand(
                        new SetSpindexPowerCommand(robot.spindexer, Math.copySign(SPINDEX_SORTING_TRANSFER_POWER, SPINDEX_TRANSFER_POWER)),
                        new SetSpindexPowerCommand(robot.spindexer, Math.copySign(transferPower, SPINDEX_TRANSFER_POWER)),
                        robot::getAutoSort
                ),


                new ConditionalCommand(
                        new WaitCommand(SPINDEX_SORTING_TRANSFER_TIME),
                        new WaitCommand(SPINDEX_TRANSFER_TIME),
                        robot::getAutoSort
                ),

                new InstantCommand(() -> robot.spindexer.setSpindexerPower(0.0)),
                new InstantCommand(() -> robot.spindexer.goToAngle120(0)),
                // reset spindexer, intake, shooter
                new ParallelCommandGroup(
                        new SetIntakeSpeedCommand(robot.intake, 0),
                        new SetSpindexerRampActive(robot.spindexer, false),
                        new SetSpindexerYawCommand(robot.spindexer, 0.0),
                        new InstantCommand(() -> robot.spindexer.setPidEnabled(true))
                ),
                new ConditionalCommand( // reverse intake briefly in tele
                        new SequentialCommandGroup(
                                new SetIntakeSpeedCommand(robot.intake,IntakeSubsystem.REVERSE_SPEED),
                                new WaitCommand(reverseIntakeTimeMS)
                        ),
                        new InstantCommand(),
                        () -> isTeleop
                ),
                new GoToRestingStateCommand(robot),
            new InstantCommand(() -> robot.spindexer.useMaxPower = false)
        );
        this.robot = robot;
    }

    public ShootThreeBallsCommand(Robot robot) {
        this(robot, SPINDEX_TRANSFER_POWER,false);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        robot.spindexer.setPidEnabled(true);
    }
}