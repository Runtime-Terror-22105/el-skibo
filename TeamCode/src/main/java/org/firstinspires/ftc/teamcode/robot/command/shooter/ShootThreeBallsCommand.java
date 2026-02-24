package org.firstinspires.ftc.teamcode.robot.command.shooter;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.LogCatCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.command.intake.SetIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.SetSpindexerRampActive;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.SetSpindexerYawCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToRestingStateCommand;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;
import org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem;

@Config
public class ShootThreeBallsCommand extends SequentialCommandGroup {
    public static double ANGLE_THRESHOLD_SPEED_CHANGE = 210;
    //    public static double SPINDEX_ROTATIONS = -4.5;  // revolutions, negative bc clockwise
    public static double SPINDEX_TRANSFER_POWER = -1;
    public static int SPINDEX_TRANSFER_TIME = 700;  // milliseconds

    private final Robot robot;

    public ShootThreeBallsCommand(Robot robot, double transferPower) {
        super(
                new InstantCommand(() -> robot.robotState = RobotState.SHOOTING),
                new SetIntakeSpeedCommand(robot.intake, IntakeSubsystem.DEFAULT_SPEED),
//                new WaitCommand(200),

                // Phase 5: transfer balls
                new InstantCommand(() -> {
                    robot.spindexer.setPidEnabled(false);
                    robot.spindexer.setSpindexerPower(Math.copySign(transferPower, SPINDEX_TRANSFER_POWER));
                }),
                new ConditionalCommand(new WaitCommand((int) (SPINDEX_TRANSFER_TIME * 2.5)),
                        new WaitCommand(SPINDEX_TRANSFER_TIME),
                        () -> robot.getAutoSort()),

                new InstantCommand(() -> robot.spindexer.setSpindexerPower(0.0)),
                new InstantCommand(() -> robot.spindexer.goToAngle120(0)),
                // reset spindexer, intake, shooter
                new ParallelCommandGroup(
                        new SetIntakeSpeedCommand(robot.intake, 0),
                        new SetSpindexerRampActive(robot.spindexer, false),
                        new SetSpindexerYawCommand(robot.spindexer, 0.0),
                        new InstantCommand(() -> robot.spindexer.setPidEnabled(true))
                ),
                new GoToRestingStateCommand(robot),
            new InstantCommand(() -> robot.spindexer.useMaxPower = false)
        );
        this.robot = robot;
    }

    public ShootThreeBallsCommand(Robot robot) {
        this(robot, SPINDEX_TRANSFER_POWER);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        robot.spindexer.setPidEnabled(true);
    }
}