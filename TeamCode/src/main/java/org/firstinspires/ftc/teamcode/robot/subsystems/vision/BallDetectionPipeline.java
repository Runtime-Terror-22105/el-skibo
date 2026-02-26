package org.firstinspires.ftc.teamcode.robot.subsystems.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortalImpl;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

public class BallDetectionPipeline implements VisionProcessor {
    //LOGIC
    //masks for color ranges, combine mask
    //do mask editing, dialation, erosion, etc
    //edge dectection- either sobel or canny
    //choose the contour one with the largest area
    //use a set multiplier to calculate pixels off center into inches off center
    //return position with the offset of the balls 

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
