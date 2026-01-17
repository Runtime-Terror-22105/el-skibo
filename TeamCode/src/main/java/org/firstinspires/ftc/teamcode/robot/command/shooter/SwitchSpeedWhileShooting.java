package org.firstinspires.ftc.teamcode.robot.command.shooter;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;

public class SwitchSpeedWhileShooting extends CommandBase {
    private final SpindexerSubsystem spindexer;
    private final double newSpeed;
    private final double angleThreshold;

    public double initialAngle;
    private boolean done;

    public SwitchSpeedWhileShooting(SpindexerSubsystem spindexer, double newSpeed, double angleThreshold) {
        this.spindexer = spindexer;
        this.newSpeed = newSpeed;
        this.angleThreshold = angleThreshold;
    }

    @Override
    public void initialize() {
        this.initialAngle = spindexer.getPosition();
        this.done = false;
    }

    @Override
    public void execute() {
        if (Angle.normalize(spindexer.getPosition() - this.initialAngle) > angleThreshold) {
            spindexer.setSpindexerPower(newSpeed);
            this.done = true;
            spindexer.overrideMaxPower = true;
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
