package org.firstinspires.ftc.teamcode.robot.hardware.motors;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.hardware.TerrorWritingDevice;

/**
 * A wrapper class for the {@link Servo}, providing position caching to avoid redundant
 * {@link Servo#setPosition(double)} calls. The class ensures the position is only updated
 * when the difference from the last set position exceeds a defined tolerance.
 */
public class TerrorServo implements TerrorWritingDevice {
    private final Servo servo;  // The underlying PhotonServo instance
    private double servoPosition;     // Current position of the servo
    private double lastPosition;      // Last set position to prevent unnecessary updates
    private final double tolerance = 0.0; // Small tolerance to avoid float comparison issues
    private ServoCommand command = ServoCommand.NONE;

    /**
     * Enum representing possible servo commands.
     */
    private enum ServoCommand {
        SET_POSITION,   // Indicates that a new position needs to be set
        NONE            // No command
    }

    /**
     * Constructs a new TerrorServo instance.
     *
     * @param servo The {@link Servo} instance to wrap.
     */
    public TerrorServo(@NonNull Servo servo) {
        this.servo = servo;
        this.servoPosition = servo.getPosition();
        this.lastPosition = -100; // prevent caching at start from being goofy
    }

    /**
     * Sets the position of the servo. The actual position update will only occur during the
     * {@link #write()} method if the difference between the new position and the last position
     * is greater than the tolerance.
     * <p>
     * This method is thread-safe.
     *
     * @param position The new position to set for the servo (range [0, 1]).
     */
    synchronized public void setPosition(double position) {
        this.servoPosition = position;
        this.command = ServoCommand.SET_POSITION;  // Indicate that a position update is required
    }

    /**
     * Sets the direction of the servo.
     * <p>
     * This method is thread-safe.
     * @param direction The {@link Servo.Direction} for the servo (e.g. FORWARD or REVERSE).
     */
    synchronized public void setDirection(Servo.Direction direction) {
        this.servo.setDirection(direction);
    }

    /**
     * Writes the position to the servo if necessary. Call this once each loop iteration.
     * <p>
     * The position will only be updated if:
     * <ol>
     *   <li>A {@link #setPosition(double)} command was issued.</li>
     *   <li>The difference between the current position and the last set position exceeds the defined tolerance of {@value #tolerance}.</li>
     * </ol>
     * This tolerance helps avoid unnecessary servo updates.
     * <p>
     * This method is thread-safe.
     */
    @Override
    synchronized public void write() {
        if (command.equals(ServoCommand.SET_POSITION) &&
                Math.abs(this.servoPosition - this.lastPosition) > this.tolerance) {
            this.lastPosition = this.servoPosition;
            this.servo.setPosition(this.servoPosition);
        }
        this.command = ServoCommand.NONE;
    }

    synchronized public double getSetPosition() {
        return this.servo.getPosition();
    }
}
