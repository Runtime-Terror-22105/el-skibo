package org.firstinspires.ftc.teamcode.robot.hardware.sensors;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

public class TerrorColorSensor implements ColorSensor {

    private final ColorSensor sensor;

    public TerrorColorSensor(@NonNull ColorSensor sensor)
    {
        this.sensor = sensor;
    }


    /**
    * returns if the color sensor sees this as G,P,orN(none)
     */
    public char getGreenOrPurple()
    {
        return //i dont want to do this rn do something about sensor.red within range and the stuff just steal my color masking or smth im tired
    }

    @Override
    public int red() {
        return sensor.red();
    }

    @Override
    public int green() {
        return sensor.green();
    }

    @Override
    public int blue() {
        return sensor.blue();
    }

    @Override
    public int alpha() {
        return sensor.alpha();
    }

    @Override
    public int argb() {
        return sensor.argb();
    }

    @Override
    public void enableLed(boolean enable) {
        sensor.enableLed(enable);
    }

    @Override
    public void setI2cAddress(I2cAddr newAddress) {
        sensor.setI2cAddress(newAddress);
    }

    @Override
    public I2cAddr getI2cAddress() {
        return sensor.getI2cAddress();
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
