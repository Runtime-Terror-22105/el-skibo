package org.firstinspires.ftc.teamcode.robot.hardware.motors;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robot.hardware.TerrorWritingDevice;

/**
 * This is a class to represent a motor connected to a REV SPARKmini instead of directly.
 * This contains much less functionality than the normal TerrorMotor class.
 */
public class TerrorSparkMiniMotor implements TerrorWritingDevice, TerrorMotor {
    private final CRServo motor;
    private TerrorMotorNormal pairedMotor;

    private final double powerThreshold;
    private final double powerScale;

    private double motorPower;
    private double lastPower;

    /**
     * Creates a new TerrorSparkMiniMotor instance.
     *
     * @param motor          The underlying CRServo that this class wraps around.
     * @param powerThreshold The threshold used to prevent unnecessary motor power updates.
     * @param powerScale     The scale to be used for the power
     */
    public TerrorSparkMiniMotor(CRServo motor, double powerThreshold, double powerScale) {
        this.powerThreshold = powerThreshold;
        this.motor = motor;
        this.lastPower = -100; // since it is outside of -1 to 1 range, it will force ignore cache
        this.motorPower = 0;
        this.powerScale = powerScale;
        this.pairedMotor = null;
    }

    public void setPairedMotor(TerrorMotorNormal pairedMotor) {
        this.pairedMotor = pairedMotor;
    }

    public double getSetPower() {
        return motorPower;
    }

    /**
     * Sets the power of the motor.
     *
     * @param power The desired power value (range: -1.0 to 1.0).
     */
    synchronized public void setPower(double power) {
        this.motorPower = power*powerScale;
    }

    synchronized public double getPower() {
        return motor.getPower();
    }

    public double getLastPower() {
        return this.lastPower;
    }

    /**
     * Sets the direction of the motor.
     *
     * @param direction The desired direction (e.g., FORWARD, REVERSE).
     */
    synchronized public void setDirection(DcMotorSimple.Direction direction) {
        this.motor.setDirection(direction);
    }

    /**
     * Retrieves the current direction of the motor.
     *
     * @return The current direction of the motor.
     */
    public DcMotorSimple.Direction getDirection() {
        return this.motor.getDirection();
    }

    /**
     * Executes the queued motor commands. This should be called each loop iteration
     * to apply any changes in motor settings such as power, velocity, or mode.
     */
    synchronized public void write() {
        if (this.pairedMotor != null && this.pairedMotor.isOverCurrent()) {
            this.motor.setPower(0.0);
            return;
        }

        // if motor power is 0 (to enforce zero power behav) or change > threshold
        if (this.motorPower == 0 || Math.abs(this.motorPower - this.lastPower) > this.powerThreshold) {
            this.lastPower = this.motorPower;
            this.motor.setPower(motorPower);
        }
    }

}
