package org.firstinspires.ftc.teamcode.robot.hardware.sensors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robot.hardware.motors.TerrorMotorNormal;

/**
 * Wraps a motor instance to provide corrected velocity counts and allow reversing independently of the corresponding
 * slot's motor direction.
 * Simply copied from the roadrunner quickstart but with some tweaks to make it simpler, also added
 * the reset() method.
 */
public class TerrorEncoder {

    public enum Direction {
        FORWARD(1),
        REVERSE(-1);

        private final int multiplier;

        Direction(int multiplier) {
            this.multiplier = multiplier;
        }

        public int getMultiplier() {
            return multiplier;
        }
    }

    public TerrorMotorNormal motor;

    private Direction direction;

    public int offset;

    private double ticksPerRevolution = -1;

    public TerrorEncoder(TerrorMotorNormal motor) {
        this.motor = motor;
        this.direction = Direction.FORWARD;
        this.offset = 0;
    }

    public TerrorEncoder(TerrorMotorNormal motor, double ticksPerRevolution) {
        this(motor);
        this.ticksPerRevolution = ticksPerRevolution;
    }

    public Direction getDirection() {
        return direction;
    }

    private int getMultiplier() {
        return getDirection().getMultiplier() * (motor.getDirection() == DcMotorSimple.Direction.FORWARD ? 1 : -1);
    }

    /**
     * Allows you to set the direction of the counts and velocity without modifying the motor's direction state
     * @param direction either reverse or forward depending on if encoder counts should be negated
     */
    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    /**
     * Gets the position from the underlying motor and adjusts for the set direction.
     * Additionally, this method updates the velocity estimates used for compensated velocity
     *
     * @return encoder position
     */
    public int getCurrentPosition() {
        int multiplier = getMultiplier();
        return motor.getCurrentPosition() * multiplier - offset;
    }

    public double getVelocity() {
        return motor.getVelocity() * getMultiplier();
    }

    public double getPositionDegrees() {
        if (this.ticksPerRevolution == -1) {
            throw new IllegalStateException("ticksPerRevolution was not set in constructor");
        }

        return getCurrentPosition() * 360.0 / this.ticksPerRevolution;
    }

//    /**
//     * Gets the velocity directly from the underlying motor and compensates for the direction
//     * See {@link #getCorrectedVelocity} for high (>2^15) counts per second velocities (such as on REV Through Bore)
//     *
//     * @return raw velocity
//     */
//    public double getVelocity() {
//        int multiplier = getMultiplier();
//        return motor.getVelocity() * multiplier;
//    }

    /**
     * I wanted to use Lynx to reset the encoder without stopping the motor but it didn't work.
     * Only run this at the start of the match, otherwise random motors will stop.
     */
    public void stop_and_reset() {
        this.offset += getCurrentPosition();
//        DcMotor.RunMode oldMode = motor.getMode();
//        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor.setMode(oldMode);
    }

    public void setMotorToMode(DcMotor.RunMode mode) {
        motor.setMode(mode);
    }
}
