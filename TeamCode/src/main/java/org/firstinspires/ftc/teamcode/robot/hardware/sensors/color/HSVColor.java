package org.firstinspires.ftc.teamcode.robot.hardware.sensors.color;

import androidx.annotation.NonNull;

public class HSVColor {
    public final float hue;
    public final float saturation;
    public final float value;

    public HSVColor(float hue, float saturation, float value) {
        this.hue = hue;
        this.saturation = saturation;
        this.value = value;
    }

    public HSVColor(@NonNull float[] color) {
        this.hue = color[0];
        this.saturation = color[1];
        this.value = color[2];
    }
}
