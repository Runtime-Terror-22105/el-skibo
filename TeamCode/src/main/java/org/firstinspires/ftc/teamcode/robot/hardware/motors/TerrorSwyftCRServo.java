package org.firstinspires.ftc.teamcode.robot.hardware.motors;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.robot.hardware.TerrorWritingDevice;
import org.firstinspires.ftc.teamcode.robot.init.Robot;

public class TerrorSwyftCRServo implements TerrorWritingDevice {
    private static final double FORWARD_POS = 1.0;
    private static final double REVERSE_POS = 0.0;
    private static final double HOME_POS = 0.5;

    public enum Direction {
        FORWARD,
        REVERSE
    }

    public enum Power {
        FORWARD(FORWARD_POS, REVERSE_POS),   // Max power forward
        REVERSE(REVERSE_POS, FORWARD_POS),   // Max power reverse
        HOME(HOME_POS, HOME_POS);  // Home the servo

        private final double forwardPos;
        private final double reversePos;

        Power(double forwardPos, double reversePos) {
            this.forwardPos = forwardPos;
            this.reversePos = reversePos;
        }

        public double getSwyftPos(Direction direction) {
            return Direction.FORWARD.equals(direction) ? forwardPos : reversePos;
        }
    }

    private final Servo servo;
    private final String debugName;
    private Direction direction = Direction.FORWARD;

    /**
     * Constructs a new TerrorServo instance.
     *
     * @param servo The {@link Servo} instance to wrap.
     */
    public TerrorSwyftCRServo(@NonNull Servo servo, String debugName) {
        this.servo = servo;
        this.debugName = debugName;
    }

    public TerrorSwyftCRServo(@NonNull HardwareMap hw, String name) {
        this(hw.get(Servo.class, name), name);
    }

    synchronized public void setDirection(Direction direction) {
        this.direction = direction;
    }

    /**
     * To disable power, use the `setPwmEnable` method. Note that setting a power value here
     * will automatically enable PWM power.
     */
    synchronized public void setPower(Power power) {
        // NB: DO NOT CACHE HERE
        double pos = power.getSwyftPos(this.direction);
        Robot.debugTelemetry.addData(debugName + " Power", power.name() + " (pos: " + pos + ")");
        this.servo.setPosition(pos);
    }

    synchronized public void setPwmEnable(boolean enabled) {
        if (enabled) {
            ((ServoImplEx) this.servo).setPwmEnable();
        } else {
            ((ServoImplEx) this.servo).setPwmDisable();
        }
    }

    @Override
    synchronized public void write() {
        // No-op
    }

    @Override
    public String debugName() {
        return this.debugName;
    }
}
