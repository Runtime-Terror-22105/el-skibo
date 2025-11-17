package org.firstinspires.ftc.teamcode.robot.command.states;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.robot.command.intake.SetIntakeDownCommand;
import org.firstinspires.ftc.teamcode.robot.command.intake.SetIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.SetSpindexerPoleActive;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.SetSpindexerRampActive;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.SetSpindexerWallActive;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;
import org.firstinspires.ftc.teamcode.robot.command.shooter.SetShooterManualAimCommand;

public class GoToRestingStateCommand extends SequentialCommandGroup {
    public GoToRestingStateCommand(Robot robot) {
        super(new ParallelCommandGroup(
                new InstantCommand(() -> robot.robotState = RobotState.RESTING),
                new SetShooterManualAimCommand(robot.shooter, 0.0, 45.0, 0.0),
                new SetIntakeDownCommand(robot.intake, false),
                new SetIntakeSpeedCommand(robot.intake, 0.0),
                new SetSpindexerPoleActive(robot.spindexer, false),
                new SetSpindexerRampActive(robot.spindexer, false)
//                new SetSpindexerWallActive(robot.spindexer, true)
        ));
    }
}
