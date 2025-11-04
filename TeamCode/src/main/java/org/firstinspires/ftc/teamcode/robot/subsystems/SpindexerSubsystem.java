package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.math.controllers.PidfController;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

public class SpindexerSubsystem extends SubsystemBase {
    private final RobotHardware hardware;

    public double spindexerOffset = 0;

    public static double INTAKE_RAMP_ACTIVE = 1.0;
    public static double INTAKE_RAMP_DEACTIVE = 0.0;
    public static double SHOOTER_RAMP_ACTIVE = 1.0;
    public static double SHOOTER_RAMP_DEACTIVE = 0.0;
    public static double WALL_ACTIVE = 1.0;
    public static double WALL_DEACTIVE = 0.0;

    public double SHOOTER_INTAKE_SPEED = 0.0; // this is the speed where the shooter melonbotic servo intakes the balls

    public static double SHOOTER_INTAKING_SPEED = 1.0;

    public double intakeRampPosition = INTAKE_RAMP_DEACTIVE;
    public double shooterRampPosition = SHOOTER_RAMP_DEACTIVE;
    public double wallPosition = WALL_DEACTIVE;

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

    public void setYaw(double angle){ //angle is in radians cuz i said so oh yeah and also have todo: optimization like the swerve pod thingy where u do the shortest distance
        this.yawPid.setTargetPosition(angle);
    }

    public boolean getLimitSwitchState() {
        return this.hardware.spindexerLimitSwitch.getState();
    }

    public void updateSpindexer(){
//        if(hardware.spindexerEncoder.getCurrentPosition())
        this.spindexerPower= yawPid.calculatePower(hardware.spindexerEncoder.getCurrentPosition()+this.spindexerOffset,0);
        // setting pid power into the spindexer
    }

    public double getPosition() {
        return hardware.spindexerEncoder.getCurrentPosition() - this.spindexerOffset;
    }

    public void activateTransfer() {
        this.intakeRampPosition = INTAKE_RAMP_ACTIVE;
        this.shooterRampPosition = SHOOTER_RAMP_ACTIVE;
        this.wallPosition = WALL_ACTIVE;
        this.SHOOTER_INTAKE_SPEED = SHOOTER_INTAKING_SPEED;
    }

    public void deactivateTransfer() {
        this.intakeRampPosition = INTAKE_RAMP_DEACTIVE;
        this.shooterRampPosition = SHOOTER_RAMP_DEACTIVE;
        this.wallPosition = WALL_DEACTIVE;
        this.SHOOTER_INTAKE_SPEED = 0.0;
    }

    public void setSpindexerOffset(double offset) {
        this.spindexerOffset = offset;
    }

    public void setSpindexerPower(double power) {
        this.spindexerPower = power;
    }

    @Override
    public void periodic() {
        this.hardware.spindexerIntakeRampServo.setPosition(this.intakeRampPosition);
        this.hardware.spindexerShooterRampServo.setPosition(this.shooterRampPosition);
        this.hardware.spindexerWallServo.setPosition(this.wallPosition);

        this.updateSpindexer();
        this.hardware.spindexerRotate.setPower(this.spindexerPower);
    }
}
