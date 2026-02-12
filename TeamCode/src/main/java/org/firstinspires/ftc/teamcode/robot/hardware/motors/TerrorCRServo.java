package org.firstinspires.ftc.teamcode.robot.hardware.motors;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.hardware.TerrorWritingDevice;

/**
 * A wrapper servo class that provides caching to avoid unnecessary setPower() calls.
 * This class manages a continuous rotation servo by minimizing calls to the underlying hardware
 * when the requested power is within a specified threshold.
 */
public class TerrorCRServo implements TerrorWritingDevice {
    private final CRServo crservo;
    private final double powerThreshold;
    private double powerScale;

    private double servoPower;
    private double lastPower;
    private CRServoCommand command = CRServoCommand.NONE;

    /**
     * Enum representing the commands that can be executed for the continuous rotation servo.
     */
    private enum CRServoCommand {
        SET_POWER, // Command to set the power of the servo
        NONE       // No command to execute
    }

    /**
     * Constructs a new TerrorCRServo.
     *
     * @param crservo         The PhotonCRServo object to wrap around.
     * @param powerThreshold  The threshold used to prevent unnecessary power updates.
     * @param powerScale      The scale for the servo power.
     */
    public TerrorCRServo(@NonNull CRServo crservo, double powerThreshold, double powerScale) {
        this.powerThreshold = powerThreshold;
        this.crservo = crservo;
        this.servoPower = 0;
        this.lastPower = -12.345; // since it is outside of -1 to 1 range, it will force ignore cache
        this.powerScale = powerScale;
    }

    public TerrorCRServo(HardwareMap hwMap, @NonNull String name, double powerThreshold, double powerScale) {
        this(hwMap.get(CRServo.class, name), powerThreshold, powerScale);
    }

    /**
     * Sets the power of the continuous rotation servo.
     * The power value should be in the range of -1.0 to 1.0.
     *
     * @param power The desired power value for the servo.
     */
    synchronized public void setPower(double power) {
        this.servoPower = power*powerScale;
        this.command = CRServoCommand.SET_POWER;
    }

    /**
     * Sets the direction of the continuous rotation servo.
     * This will configure the direction in which the servo will turn.
     *
     * @param direction The desired direction (e.g., FORWARD or REVERSE).
     */
    synchronized public void setDirection(DcMotorSimple.Direction direction) {
        this.crservo.setDirection(direction);
    }

    /**
     * Writes the cached commands to the actual servo.
     * This method should be called each loop iteration.
     * The power will only be updated if the change exceeds the specified power threshold.
     */
    synchronized public void write() {
        if (command.equals(CRServoCommand.SET_POWER) &&
                Math.abs(this.servoPower - this.lastPower) > this.powerThreshold) {
            this.crservo.setPower(servoPower);
            this.lastPower = servoPower;
        }
        this.command = CRServoCommand.NONE;
    }


    synchronized public void setScale(double scale) {
        this.powerScale = scale;
    }
}
