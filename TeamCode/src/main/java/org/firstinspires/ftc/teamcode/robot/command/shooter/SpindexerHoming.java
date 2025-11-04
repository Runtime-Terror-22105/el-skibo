package org.firstinspires.ftc.teamcode.robot.command.shooter;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;

public class SpindexerHoming extends CommandBase {

    private final SpindexerSubsystem spindexer;
    private boolean homed;
    private boolean seenSwitchStart;
    private double switchStart;
    private double switchEnd;
    private double initialPosition;

    private static final double MAX_ROTATION_DEGREES = 120.0;
    private static final double POWER = 0.5;

    public SpindexerHoming(SpindexerSubsystem spindexer) {
        this.spindexer = spindexer;
        addRequirements(spindexer);
    }

    @Override
    public void initialize() {
        homed = false;
        seenSwitchStart = false;
        switchStart = Double.NaN;
        switchEnd = Double.NaN;
        initialPosition = spindexer.getPosition();
        // start moving toward the limit switch
        spindexer.setSpindexerPower(POWER);
    }

    @Override
    public void execute() {
        if (homed) return;

        double pos = spindexer.getPosition();
        boolean limit = spindexer.getLimitSwitchState();

        // detect start of the true-range
        if (!seenSwitchStart && limit) {
            seenSwitchStart = true;
            switchStart = pos;
        }

        // detect end of the true-range (true -> false transition)
        if (seenSwitchStart && !Double.isNaN(switchStart) && !limit && Double.isNaN(switchEnd)) {
            switchEnd = pos;
            finalizeHoming();
            return;
        }

        // time or rotation safeguard
        boolean rotatedEnough = Math.abs(pos - initialPosition) >= MAX_ROTATION_DEGREES;
        if (rotatedEnough) {
            // if we saw the start but never saw the end, use current pos as end
            if (seenSwitchStart && Double.isNaN(switchEnd)) {
                switchEnd = pos;
            }
            // if we never saw the switch at all, use current position as zero fallback
            if (!seenSwitchStart) {
                spindexer.setSpindexerOffset(pos);
                spindexer.setSpindexerPower(0.0);
                homed = true;
                return;
            }
            finalizeHoming();
        }
        // otherwise keep spinning until we actually find it
    }

    private void finalizeHoming() {
        double start = Double.isNaN(switchStart) ? spindexer.getPosition() : switchStart;
        double end = Double.isNaN(switchEnd) ? spindexer.getPosition() : switchEnd;
        double avg = (start + end) / 2.0;
        spindexer.setSpindexerOffset(avg);
        spindexer.setSpindexerPower(0.0);
        homed = true;
    }

    @Override
    public boolean isFinished() {
        return homed;
    }

}