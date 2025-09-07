package org.firstinspires.ftc.teamcode.robot.hardware.sensors;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TerrorProximitySensor {
    private final AnalogInput sensor;
    private final double threshold;

    public TerrorProximitySensor(@NonNull HardwareMap hw, String name, double threshold) {
        this.sensor = hw.get(AnalogInput.class, name);
        this.threshold = threshold;
    }

    public double getValue() {
        return sensor.getVoltage();
    }
}
