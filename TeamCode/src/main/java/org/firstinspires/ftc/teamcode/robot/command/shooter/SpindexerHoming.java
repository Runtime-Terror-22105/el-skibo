package org.firstinspires.ftc.teamcode.robot.command.shooter;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;

public class SpindexerHoming extends CommandBase {

    private final SpindexerSubsystem spindexer;
    private boolean homed;

    public SpindexerHoming(SpindexerSubsystem spindexer) {
        this.spindexer = spindexer;
    }

    @Override
    public void initialize() {
        homed = false;
    }

    @Override
    public void execute() {
        if (!spindexer.getLimitSwitchState()) {
            spindexer.setSpindexerPower(0.5);
        } else if (!homed) {
            spindexer.setSpindexerOffset(spindexer.getPosition());
            spindexer.setSpindexerPower(0.0);
            homed = true;
        }
    }

    @Override
    public boolean isFinished() {
        return homed;
    }

}
