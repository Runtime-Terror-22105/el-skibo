package org.firstinspires.ftc.teamcode.robot.hardware.sensors;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.broadcom.BatchColorSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.BallColor;

@Config
public class TerrorColorSensor implements NormalizedColorSensor {
    public static double MAX_DIST = 50;

    private final RevColorSensorV3 sensor;
    private final BatchColorSensor reading = new BatchColorSensor();

    public TerrorColorSensor(@NonNull RevColorSensorV3 sensor) {
        this.sensor = sensor;
    }

    public void reset() {
        reading.reset();
    }

    public void update() {
        reading.read(sensor);
    }

    public boolean hasFullData() {
        return reading.hasFullData();
    }

    public double getRed() {
        return reading.red();
    }

    public double getGreen() {
        return reading.green();
    }

    public double getBlue() {
        return reading.blue();
    }

    public double getDist(DistanceUnit unit) {
        return unit.fromUnit(DistanceUnit.INCH, reading.distance());
    }

    public BallColor getBallColor() {
        double r = getRed();
        double g = getGreen();
        double b = getBlue();
//        Log.d("Color-sensor", getDist(DistanceUnit.MM) + " " + g);
        if (getDist(DistanceUnit.MM) > MAX_DIST) {
            return BallColor.NONE;
        } else if (b > g && b > r) {
            return BallColor.PURPLE;
        } else if (g > b && g > r) {
            return BallColor.GREEN;
        }
        return BallColor.NONE;
    }

    public NormalizedRGBA getNormalizedColors() {
        return sensor.getNormalizedColors();
    }

    @Override
    public float getGain() {
        return 0;
    }

    @Override
    public void setGain(float newGain) {

    }

    @Override
    public Manufacturer getManufacturer() {
        return sensor.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return sensor.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return sensor.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return sensor.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        sensor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        sensor.close();
    }
}
