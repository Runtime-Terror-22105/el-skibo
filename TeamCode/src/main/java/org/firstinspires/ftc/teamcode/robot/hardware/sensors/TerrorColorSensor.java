package org.firstinspires.ftc.teamcode.robot.hardware.sensors;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class TerrorColorSensor implements NormalizedColorSensor {

    private final RevColorSensorV3 sensor;

    public TerrorColorSensor(@NonNull RevColorSensorV3 sensor)
    {
        this.sensor = sensor;
    }


    /**
    * returns if the color sensor sees this as G,P,orN(none)
     */
    public double getGreen(){
        return ((double)sensor.green());
    }

    public double getRed(){
        return ((double)sensor.red());
    }

    public double getBlue(){
        return ((double)sensor.blue());
    }



    public double getDist(DistanceUnit unit){
        return sensor.getDistance(unit);
    }

    public char getGreenOrPurple() {

        double[]rgb= {getRed(),getGreen(),getBlue()};
        if(getDist(DistanceUnit.MM)>=30.0){
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
