package org.firstinspires.ftc.teamcode.robot.command.spindexer;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;

public class WaitForSpindexerYawCommand extends WaitUntilCommand {
    public static ElapsedTime timer = null; // todo: having this static is sus, this approach should probably be better
    public static boolean timerStart = false; // todo: having this static is sus, this approach should probably be better

    public WaitForSpindexerYawCommand(SpindexerSubsystem spindexer) {
        super(spindexer::atTargetYaw);
    }

    public WaitForSpindexerYawCommand(SpindexerSubsystem spindexer, double reachedTimeMillis) {
        super(() -> {
            if (timer == null) {
                timer = new ElapsedTime();
                timer.reset();
            }

            if (spindexer.atTargetYaw()) {
                if (!timerStart) {
                    timerStart = true;
                    timer.reset();
                } else if (timer.milliseconds() > reachedTimeMillis) {
                    return true;
                }
            } else {
                timer.reset();
            }

            return false;
        });
    }
}
