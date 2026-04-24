package org.firstinspires.ftc.teamcode.robot.command.states;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.robot.command.intake.SetIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.robot.command.intake.SetIntakeUpCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.SetBallBlockerActiveCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.SetSpindexerRampActive;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.SetSpindexerWallDown;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;

public class GoToRestingStateCommand extends SequentialCommandGroup {
    public GoToRestingStateCommand(Robot robot) {
        super(new ParallelCommandGroup(
                new InstantCommand(() -> robot.robotState = RobotState.RESTING),
                new SetIntakeSpeedCommand(robot.intake, 0.0),
                new SetSpindexerRampActive(robot.spindexer, false),
                new SetSpindexerWallDown(robot.spindexer, true),
                new SetIntakeUpCommand(robot.intake, false),
                new SetBallBlockerActiveCommand(robot.spindexer, true)
        ));
    }
}
