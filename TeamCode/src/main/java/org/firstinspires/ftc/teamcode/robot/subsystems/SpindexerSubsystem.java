package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.math.controllers.PidfController;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@Config
public class SpindexerSubsystem extends SubsystemBase {
    public static double RAMP_DOWN_POSITION = 1.0; // transfer ramp down
    public static double RAMP_UP_POSITION = 0.0; // transfer ramp up

    private final RobotHardware hardware;

    private double rampPosition = RAMP_UP_POSITION;

    public double spindexerPower = 0.0;
    public static PidfController.PidfCoefficients turningPidCoefficients =
            new PidfController.PidfCoefficients(0.014, 0, 0, 1, 0);
    public static double yawPidTolerance = 0.1;
    public final PidfController yawPid = new PidfController(turningPidCoefficients);

    public SpindexerSubsystem(RobotHardware hardware) {
        this.hardware = hardware;

        this.yawPid.setTolerance(yawPidTolerance);
        this.yawPid.setTargetPosition(0.0);
    }

    public double getTargetYaw() {
        return this.yawPid.getTargetPosition();
    }

    public void setYaw(double angle) { //angle is in radians cuz i said so oh yeah and also have todo: optimization like the swerve pod thingy where u do the shortest distance
        this.yawPid.setTargetPosition(angle);
    }

    public void updateSpindexer(){
//        if(hardware.spindexerEncoder.getCurrentPosition())
        this.spindexerPower = yawPid.calculatePower(hardware.spindexerEncoder.getCurrentPosition(),0);
        // setting pid power into the spindexer
    }

    public double getPosition(){
        return hardware.spindexerEncoder.getCurrentPosition();
    }

    public void activateTransfer() {
        this.rampPosition = SpindexerSubsystem.RAMP_DOWN_POSITION;
    }

    public void deactivateTransfer() {
        this.rampPosition = SpindexerSubsystem.RAMP_UP_POSITION;
    }

    @Override
    public void periodic() {
        this.hardware.spindexerTransferRamp.setPosition(this.rampPosition);
        this.updateSpindexer();
        this.hardware.spindexerRotate.setPower(this.spindexerPower);
    }
}
