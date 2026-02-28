/**
 * Note: All the functions related to setting exposure originate from ConceptAprilTagOptimizeExposure by FIRST
 * (but are modified a bit for our purposes).
 */
package org.firstinspires.ftc.teamcode.robot.subsystems.vision;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

public class CameraUtil {
    private static final String TAG = "CameraUtil";

    // wait like 50 ms after calling this function to call setManualExposure() to make sure the camera is in manual mode before setting the exposure and gain
    static boolean setManualExposureMode(VisionPortal visionPortal) {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            Log.d(TAG, "setManualExposureMode: Vision Portal is null");
            return false;
        }

        // Wait for the camera to be open
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            Log.d(TAG, "setManualExposureMode: Camera is not streaming");
            return false;
        }

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        return true;
    }

    static boolean setManualExposure(VisionPortal visionPortal, int exposureMS, int gain) {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            return false;
        }

        // Wait for the camera to be open
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return false;
        }

        // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            return false; // call the function again to set the exposure and gain after we are in manual mode
            // note that you need to wait like 50 ms
        }
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);

        // Set Gain.
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        return true;
    }

    public static class CameraSetting {
        public int minExposure;
        public int maxExposure;
        public int minGain;
        public int maxGain;

        public CameraSetting() {

        }

        public CameraSetting(int minExposure, int maxExposure, int minGain, int maxGain) {
            this.minExposure = minExposure;
            this.maxExposure = maxExposure;
            this.minGain = minGain;
            this.maxGain = maxGain;
        }
    }

    /*
        Read this camera's minimum and maximum Exposure and Gain settings.
        Can only be called AFTER calling initAprilTag();
     */
    static CameraSetting getCameraSetting(VisionPortal visionPortal) {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            return null;
        }

        // Wait for the camera to be open
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return null;
        }

        CameraSetting setting = new CameraSetting();

        // Get camera control values unless we are stopping.
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        setting.minExposure = (int)exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
        setting.maxExposure = (int)exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        setting.minGain = gainControl.getMinGain();
        setting.maxGain = gainControl.getMaxGain();

        return setting;
    }
}
