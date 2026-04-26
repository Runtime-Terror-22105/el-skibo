package org.firstinspires.ftc.teamcode.robot.hardware.sensors;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.math.Angle;

@Config
public class TerrorAnalogEncoder {
    private double offset = 0;
    private final AnalogInput encoder;
    private final boolean reversed;
    private double lastPos;
    private final double voltageMin;
    private final double voltageMax;

    private static double MULTIPLIER_OFFSET =1;

    public TerrorAnalogEncoder(AnalogInput encoder, boolean reversed, double voltageMin, double voltageMax) {
        this.encoder = encoder;
        this.reversed = reversed;
        this.voltageMin = voltageMin;
        this.voltageMax = voltageMax;
    }

    public double getCurrentPositionWithoutOffset() {
        double pos = ((encoder.getVoltage() - voltageMin) / (voltageMax-voltageMin)) * Math.PI*2;
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
        double pos = ((Math.min(Math.max(encoder.getVoltage(), voltageMin), voltageMax) - voltageMin) / (voltageMax-voltageMin))
                * Math.PI*2* MULTIPLIER_OFFSET + offset;
        if (reversed) {
            pos = 2*Math.PI - pos;
        }
        pos = Angle.normalize(pos);
        return pos;
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
