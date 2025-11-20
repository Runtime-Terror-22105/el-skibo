package org.firstinspires.ftc.teamcode.robot.hardware.sensors;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.broadcom.BatchColorSensor;
import com.qualcomm.hardware.broadcom.BroadcomColorSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.nio.ByteOrder;
import java.util.Arrays;

public class TerrorColorSensor implements NormalizedColorSensor {

    public static double MAX_DIST = 50;

    private final RevColorSensorV3 sensor;
    public enum side {
        LEFT,
        TOP,
        RIGHT
    }
    public side position;

    private BatchColorSensor reading = new BatchColorSensor();

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

    public void update() {
        reading.read(sensor);
    }

    /**
    * returns if the color sensor sees this as G,P,orN(none)
     */
    public double getRed(){
        return reading.red;
    }

    public double getGreen(){
        return reading.green;
    }

    public double getBlue(){
        return reading.blue;
    }

    public double getDist(DistanceUnit unit){
//        return sensor.getDistance(unit);
        return unit.fromUnit(DistanceUnit.INCH, reading.distance);
    }

    public char getGreenOrPurple() {

        double[]rgb= {getRed(),getGreen(),getBlue()};
        if(getDist(DistanceUnit.MM) >= MAX_DIST){
            return 'N';
        }
        else if(rgb[2]>rgb[1] && rgb[2]>rgb[0]){
            return 'P';
        } else if (rgb[1]>rgb[2]&&rgb[1]>rgb[0]) {
            return 'G';
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
