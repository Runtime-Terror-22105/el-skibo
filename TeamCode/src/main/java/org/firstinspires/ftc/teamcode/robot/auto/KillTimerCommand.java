package org.firstinspires.ftc.teamcode.robot.auto;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.init.Robot;

// Kills the robot's drive motors after 29.5 seconds.
public class KillTimerCommand extends SequentialCommandGroup {
    public KillTimerCommand(Robot robot) {
        addCommands(
                new WaitCommand(29500),
                new InstantCommand(() -> robot.drive.killMotors = true)
        );
    }
}
