package com.qualcomm.hardware.broadcom;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorUtil;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.TypeConversion;

import java.nio.ByteOrder;
import java.util.Arrays;

/**
 * @author lunbun
 */
public class BatchColorSensor {
    public boolean hasData = false;
    private int red = 0, green = 0, blue = 0;
    private double distance = Double.MAX_VALUE;

    public void reset() {
        hasData = false;
        red = 0;
        green = 0;
        blue = 0;
        distance = Double.MAX_VALUE;
    }

    public void read(RevColorSensorV3 sensor) {
        byte[] regs = sensor.read(BroadcomColorSensor.Register.MAIN_STATUS, 15);

        byte mainStatus = regs[0];

        byte[] rawOpticalBytes = Arrays.copyOfRange(regs, 1, 3);
        int rawOptical = TypeConversion.unsignedShortToInt(TypeConversion.byteArrayToShort(rawOpticalBytes, 0, ByteOrder.LITTLE_ENDIAN)) & 2047;
        distance = RevColorSensorUtil.inFromOptical(sensor, rawOptical);

        if ((mainStatus & BroadcomColorSensor.MainStatus.LS_DATA_STATUS.bVal) != 0) {
            byte[] rawColorBytes = Arrays.copyOfRange(regs, 6, 15);

            green = TypeConversion.unsignedShortToInt(TypeConversion.byteArrayToShort(rawColorBytes, 0, ByteOrder.LITTLE_ENDIAN));
            blue = Range.clip((int)(1.55 * (double)TypeConversion.unsignedShortToInt(TypeConversion.byteArrayToShort(rawColorBytes, 3, ByteOrder.LITTLE_ENDIAN))), 0, 65535);
            red = Range.clip((int)(1.07 * (double)TypeConversion.unsignedShortToInt(TypeConversion.byteArrayToShort(rawColorBytes, 6, ByteOrder.LITTLE_ENDIAN))), 0, 65535);
//            int alpha = (red + green + blue) / 3;
//            BroadcomColorSensorImpl sensorImpl = sensor;
//            float r = Range.clip((float)red * sensorImpl.softwareGain / (float)(sensor.getParameters()).colorSaturation, 0.0F, 1.0F);
//            float g = Range.clip((float)green * sensorImpl.softwareGain / (float)(sensor.getParameters()).colorSaturation, 0.0F, 1.0F);
//            float b = Range.clip((float)blue * sensorImpl.softwareGain / (float)(sensor.getParameters()).colorSaturation, 0.0F, 1.0F);
//            float avg = (float)(red + green + blue) / 3.0F;
//            float a = (float)(-(65535.0 / (Math.pow(avg, 2.0) + 65535.0)) + 1.0);
//
//            float scale = 256.0F;
//            int min = 0;
//            int max = 255;
//            argb = Color.argb(Range.clip((int)(a * scale), min, max), Range.clip((int)(r * scale), min, max), Range.clip((int)(g * scale), min, max), Range.clip((int)(b * scale), min, max));
        }

        hasData = true;
    }

    private void assertHasData() {
        if (!hasData) {
            throw new IllegalStateException("No color sensor data available. Did you forget to call update()?");
        }
    }

    public int red() {
        assertHasData();
        return red;
    }

    public int green() {
        assertHasData();
        return green;
    }

    public int blue() {
        assertHasData();
        return blue;
    }

    public double distance() {
        assertHasData();
        return distance;
    }
}
