package org.firstinspires.ftc.teamcode.robot.hardware.sensors;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.datastructures.CircularBuffer;

@Config
public class TerrorAnalogEncoder {
    public static double ANGLE_120_MIN = 0;
    public static double ANGLE_240_MIN = 0;

    public static double VOLTAGE_MIN = 0.017;
    public static double VOLTAGE_MAX = 3.211;

    private double offset = 0;
    private final AnalogInput encoder;
    private final boolean reversed;
    private double lastPos;
    private final ElapsedTime timer;
//    private final CircularBuffer<Double> lastReads;

//    private static double multilpier_offset=90/84;
    private static double multilpier_offset=1;

    public TerrorAnalogEncoder(AnalogInput encoder, boolean reversed) {
        this.encoder = encoder;
        this.reversed = reversed;
        this.timer = new ElapsedTime();
    }

    public double getCurrentPositionWithoutOffset() {
        double pos = ((encoder.getVoltage() - VOLTAGE_MIN) / (VOLTAGE_MAX-VOLTAGE_MIN)) * Math.PI*2;
        if (reversed) {
            pos = 2*Math.PI - pos;
        }
        pos = Angle.normalize(pos);
        return pos;
    }

    /**
     * Returns the CURRENT position of the servo from the absolute encoder.
     * @return The absolute position.
     */
    public double getCurrentPosition() {
        double pos = ((Math.min(Math.max(encoder.getVoltage(), VOLTAGE_MIN), VOLTAGE_MAX) - VOLTAGE_MIN) / (VOLTAGE_MAX-VOLTAGE_MIN)) * Math.PI*2*multilpier_offset + offset;
        if (reversed) {
            pos = 2*Math.PI - pos;
        }
        pos = Angle.normalize(pos);
        return pos;
    }

    public double getCurrentVelocity() {
        double pos = getCurrentPosition();
        double vel = (pos - lastPos)/(timer.seconds());
        timer.reset();
        this.lastPos = pos;

        return vel;
    }

    /**
     * Returns the raw voltage the port is giving. Only for debugging.
     * @return The voltage read at the analog port.
     */
    public double getVoltage() {
        return this.encoder.getVoltage();
    }

    /**
     * Sets an offset to be added to the return value of getPosition()
     * @param offset The offset
     */
    public void setOffset(double offset) {
        this.offset = offset;
    }
}
