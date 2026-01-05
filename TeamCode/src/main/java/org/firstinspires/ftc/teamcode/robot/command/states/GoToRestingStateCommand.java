package org.firstinspires.ftc.teamcode.robot.command.states;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.robot.command.intake.SetIntakePitchCommand;
import org.firstinspires.ftc.teamcode.robot.command.intake.SetIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.SetSpindexerRampActive;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.SetSpindexerWallDown;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;
import org.firstinspires.ftc.teamcode.robot.command.shooter.SetShooterManualAimCommand;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake.IntakePitch;

public class GoToRestingStateCommand extends SequentialCommandGroup {
    public GoToRestingStateCommand(Robot robot) {
        super(new ParallelCommandGroup(
                new InstantCommand(() -> robot.robotState = RobotState.RESTING),
//                new SetShooterManualAimCommand(robot.shooter, 400, 45.0, 0.0),
                new SetIntakePitchCommand(robot.intake, IntakePitch.UP),
                new SetIntakeSpeedCommand(robot.intake, 0.0),
                new SetSpindexerRampActive(robot.spindexer, false),
                new SetSpindexerWallDown(robot.spindexer, false)
        ));
    }
}
