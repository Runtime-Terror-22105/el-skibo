package org.firstinspires.ftc.teamcode.robot.hardware.sensors.color;

import android.graphics.Color;

import androidx.annotation.ColorInt;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.match.SampleColor;
import org.jetbrains.annotations.Contract;

@Config
public class TerrorRevColorSensor {

    public static double DISTANCE_THRESHOLD_FOR_SAMPLE = 31;

    public final TerrorRevColorSensorV3 sensor;

    public TerrorRevColorSensor(TerrorRevColorSensorV3 colorSensor) {
        this.sensor = colorSensor;
    }

    public NormalizedRGBA getNormalizedRGBA() {
        return this.sensor.getNormalizedColors();
    }

    public HSVColor getColorsHSV() {
        return rgbaToHSV(this.rgb());
    }

    @Contract("_ -> new")
    @NonNull
    private static HSVColor rgbaToHSV(@NonNull int[] rgb) {
        float[] hsv = new float[3];
        Color.RGBToHSV(rgb[0], rgb[1], rgb[2], hsv);
        return new HSVColor(hsv);
    }

    public void enableLed(boolean enableLed) {
        sensor.enableLed(enableLed);
    }

    public int red() {
        return sensor.red();
    }

    public int green() {
        return sensor.green();
    }

    public int blue() {
        return sensor.blue();
    }

    public int alpha() {
        return sensor.alpha();
    }

    public int[] rgb() {
        return new int[]{red(), blue(), green()};
    }

    public int[] rgba() {
        return new int[]{red(), blue(), green(), alpha()};
    }

    public @ColorInt int argb() {
        return sensor.argb();
    }

    public SampleColor getColor(boolean isBeingHeld) {
        if (!isBeingHeld) {
            return SampleColor.NONE;
        } else if (blue() > 280) {
            return SampleColor.BLUE;
        } else if (red() > 215 && green() < 300) {
            return SampleColor.RED;
        } else {
            return SampleColor.YELLOW;
        }
    }

    public double getDistance(DistanceUnit unit) {
        return sensor.getDistance(unit);
    }

    public boolean hasSample() {
        return hasSample(DISTANCE_THRESHOLD_FOR_SAMPLE);
    }

    public boolean hasSample(double threshold) {
        return sensor.getDistance(DistanceUnit.MM) < threshold;
    }
}
