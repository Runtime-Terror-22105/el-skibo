package com.qualcomm.hardware.rev;

/**
 * @author lunbun
 */
public class RevColorSensorUtil {
    public static double getAParam(RevColorSensorV3 sensor) {
        return sensor.aParam;
    }

    public static double getBInvParam(RevColorSensorV3 sensor) {
        return sensor.binvParam;
    }

    public static double getCParam(RevColorSensorV3 sensor) {
        return sensor.cParam;
    }

    public static double getMaxDist(RevColorSensorV3 sensor) {
        return sensor.maxDist;
    }

    public static double inFromOptical(RevColorSensorV3 sensor, int rawOptical) {
        if ((double)rawOptical <= getCParam(sensor)) {
            return getMaxDist(sensor);
        } else {
            double dist = Math.pow(((double)rawOptical - getCParam(sensor)) / getAParam(sensor), getBInvParam(sensor));
            return Math.min(dist, getMaxDist(sensor));
        }
    }
}
