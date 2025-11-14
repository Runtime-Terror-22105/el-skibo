package org.firstinspires.ftc.teamcode.robot.hardware.sensors;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class TerrorColorSensor implements NormalizedColorSensor {

    private final RevColorSensorV3 sensor;
    public enum side {
        LEFT,
        TOP,
        RIGHT
    }
    public side position;

    public TerrorColorSensor(@NonNull RevColorSensorV3 sensor)
    {

        this.sensor = sensor;
        if (this.sensor.getDeviceName() == "topSensor"){
            this.position = side.TOP;
        }
        else if (this.sensor.getDeviceName() == "rightSensor"){
            this.position = side.RIGHT;
        }
        else if (this.sensor.getDeviceName() == "leftSensor"){
            this.position = side.LEFT;
        }
    }


    /**
    * returns if the color sensor sees this as G,P,orN(none)
     */
    public double getGreen(){
        return ((double)sensor.green())/65535.0*255.0;
    }

    public double getRed(){
        return ((double)sensor.red())/65535.0*255.0;
    }

    public double getBlue(){
        return ((double)sensor.blue())/65535.0*255.0;
    }

    public double getDist(DistanceUnit unit){
        return sensor.getDistance(unit);
    }

    public char getGreenOrPurple() {

        float[] hsv = new float[3];

        Color.RGBToHSV(
                sensor.red(),
                sensor.green(),
                sensor.blue(),
                hsv
        );

        float hue = hsv[0];
        float sat = hsv[1];
        float val = hsv[2];

//        // low brightness/saturation = probs nothing
        if (val < 0.15f || sat < 0.2f) {
            return 'N';
        }

        if (hue >= 80f && hue <= 160f) { // green hue range (~80–160 deg)
            return 'G';
        }

        if (hue >= 250f && hue <= 320f) { // purple hue range (~250–320 deg)
            return 'P';
        }

        return 'N';
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
