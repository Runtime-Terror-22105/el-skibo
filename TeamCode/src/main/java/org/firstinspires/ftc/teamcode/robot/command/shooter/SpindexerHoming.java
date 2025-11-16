package org.firstinspires.ftc.teamcode.robot.command.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;
import android.util.Log;

@Config
public class SpindexerHoming extends CommandBase {

    private final SpindexerSubsystem spindexer;
    private boolean homed = false;
    public boolean limit = false;
    public boolean seenSwitchStart = false;
    public double switchStart = Double.NaN;
    public double switchEnd = Double.NaN;
    private double initialPosition = 0.0;

    public static double MAX_ROTATION_TICKS = 1200.0;
    public static double POWER = 0.2;

    public SpindexerHoming(SpindexerSubsystem spindexer) {
        this.spindexer = spindexer;
        addRequirements(spindexer);
    }

    @Override
    public void initialize() {
        homed = false;
        limit = false;
        seenSwitchStart = false;
        switchStart = Double.NaN;
        switchEnd = Double.NaN;
        initialPosition = spindexer.getPosition();
        // start moving toward the limit switch
        spindexer.setPidEnabled(false);
        spindexer.setSpindexerPower(POWER);
    }

    @Override
    public void execute() {
        if (homed) return;

        // todo: check if this wraps around, bc if so then that will cause the rotatedenough condition to not work
        double pos = spindexer.getPosition();
        limit = spindexer.getLimitSwitchState();

        // detect start of the true-range
        if (!seenSwitchStart && limit) {
            seenSwitchStart = true;
            switchStart = pos;
        }

        // detect end of the range where the switch returns true
        if (seenSwitchStart && !Double.isNaN(switchStart) && !limit && Double.isNaN(switchEnd)) {
            switchEnd = pos;
            finalizeHoming();
            return;
        }

        // time or rotation safeguard
        boolean rotatedEnough = Math.abs(pos - initialPosition) >= MAX_ROTATION_TICKS;
        if (rotatedEnough) {
            // if we saw the start but never saw the end, use current pos as end
            if (seenSwitchStart && Double.isNaN(switchEnd)) {
                switchEnd = pos;
            }
            // if we never saw the switch at all, use current position as zero fallback
            if (!seenSwitchStart) {
                spindexer.setSpindexerOffset(pos);
                spindexer.setSpindexerPower(0.0);
            }
            homed = true;
            finalizeHoming();
            return;
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
    public void end(boolean interrupted) {
        spindexer.setSpindexerPower(0.0);
        spindexer.setPidEnabled(true);
        spindexer.setYaw(0.0);
        Log.d("homing","reached the end portion where we set the pid to enable and we set the yaw to 0");
    }

    @Override
    public boolean isFinished() {
        return homed;
    }

}