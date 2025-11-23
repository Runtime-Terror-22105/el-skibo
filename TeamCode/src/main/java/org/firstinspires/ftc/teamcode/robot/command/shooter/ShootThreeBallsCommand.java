package org.firstinspires.ftc.teamcode.robot.command.shooter;

import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.robot.command.intake.SetIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.SetSpindexerPoleActive;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.SetSpindexerRampActive;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.SetSpindexerYawCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.WaitForSpindexerYawCommand;
import org.firstinspires.ftc.teamcode.robot.init.Robot;

public class ShootThreeBallsCommand extends SequentialCommandGroup {
    public ShootThreeBallsCommand(Robot robot) {
        super(
                new ParallelRaceGroup( // keep going for either 2 rotations or until all balls are gone
                        new WaitForSpindexerYawCommand(robot.spindexer)
//                        new WaitUntilCommand(() -> {
//                            char[] balls = robot.spindexer.getBallPositions();
//                            return balls[0] == 'N' && balls[1] == 'N' && balls[2] == 'N';
//                        })
                ),

                // reset spindexer, intake, shooter, and pole
                new ParallelCommandGroup(
                        new SetIntakeSpeedCommand(robot.intake, 0),
                        new SetSpindexerPoleActive(robot.spindexer, false),
                        new SetSpindexerRampActive(robot.spindexer, false),
                        new SetSpindexerYawCommand(robot.spindexer, 0.0)
                )
        );
    }
}