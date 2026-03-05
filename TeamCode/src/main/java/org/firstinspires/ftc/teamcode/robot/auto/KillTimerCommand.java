package org.firstinspires.ftc.teamcode.robot.auto;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.init.Robot;

// Kills the robot's drive motors after 29.5 seconds.
@Config
public class KillTimerCommand extends SequentialCommandGroup {
    public static int KILL_TIME = 29500;

    public KillTimerCommand(Robot robot) {
        addCommands(
                new WaitCommand(KILL_TIME),
                new InstantCommand(() -> robot.drive.killMotors = true)
        );
    }
}
