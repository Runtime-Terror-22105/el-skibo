package org.firstinspires.ftc.teamcode.robot.hardware.sensors.color;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.robot.match.SampleColor;

public class TerrorColorRangeFinder {
    private DigitalChannel d_pin0;
    private DigitalChannel d_pin1;

    private AnalogInput a_pin0;

    private enum SensorMode {
        DIGITAL,
        ANALOG
    }
    private final SensorMode mode;

    public TerrorColorRangeFinder(@NonNull DigitalChannel pin0, @NonNull DigitalChannel pin1) {
        this.d_pin0 = pin0;
        this.d_pin1 = pin1;
        this.mode = SensorMode.DIGITAL;
    }

    public TerrorColorRangeFinder(@NonNull AnalogInput pin0) {
        this.a_pin0 = pin0;
        this.mode = SensorMode.ANALOG;
    }

    public SampleColor getColor() {
        if (mode == SensorMode.DIGITAL) {
            boolean pin0state = d_pin0.getState();
            boolean pin1state = d_pin1.getState();

            if (pin0state && pin1state) {
                return SampleColor.YELLOW;
            } else if (pin0state) {
                return SampleColor.RED;
            } else if (pin1state) {
                return SampleColor.BLUE;
            } else {
                return SampleColor.NONE;
            }
        } else if (mode == SensorMode.ANALOG) {
            // TODO: idk!!!
            return SampleColor.NONE;
        }
        return SampleColor.NONE;
    }

    public boolean matchesColor(@NonNull SampleColor[] colors) {
        SampleColor currentColor = this.getColor();
        for (SampleColor color : colors) {
            if (color.equals(currentColor)) {
                return true;
            }
        }
        return false;
    }

    public static boolean matchesColor(@NonNull SampleColor[] colors, @NonNull SampleColor otherColor) {
        for (SampleColor color : colors) {
            if (color.equals(otherColor)) {
                return true;
            }
        }
        return false;
    }
}

