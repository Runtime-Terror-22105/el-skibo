package org.firstinspires.ftc.teamcode.robot.subsystems.vision;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.imgproc.Moments;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

@Config
public class RampPipeline implements VisionProcessor
{
    private final Mat hsv = new Mat();

    public static double acceptPixelsAbove = 820;

//    Mat purpleMask1 = new Mat();
//    Mat purpleMask2 = new Mat();
    Mat purpleMask = new Mat();

    Mat greenMask = new Mat();

    private Mat roiMask = new Mat();

    public static volatile double MIN_AREA = 50; // tune this for noise filtering

    public static Point[] ROI_POINTS = {
            new Point(40,100),
            new Point(40,140),
            new Point(50,140),
            new Point(50,180),
            new Point(280,180),
            new Point(250,70),
    };

    public int ballsInRamp = 0;

    private void setBallsInRamp(int amount)
    {
        this.ballsInRamp = amount;
    }
    private final List<Point> detectedCenters = new ArrayList<>();
    Telemetry telemetry;

    public RampPipeline() {

    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        roiMask = new Mat(height, width, 0);

        MatOfPoint maskShape = new MatOfPoint();
        maskShape.fromArray(ROI_POINTS);

        List<MatOfPoint> polygons = new ArrayList<>();
        polygons.add(maskShape);
        Imgproc.fillPoly(roiMask, polygons, new Scalar(255));
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {


        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);
//        Core.inRange(hsv, purpleLow1, purpleHigh1, purpleMask1);
        Core.inRange(hsv, ColorRange.purpleLow, ColorRange.purpleHigh, purpleMask);
//        Core.bitwise_or(purpleMask1, purpleMask2, purpleMask);
        Core.inRange(hsv, ColorRange.greenLow, ColorRange.greenHigh, greenMask);

        Mat combinedMask = new Mat();
        Core.bitwise_or(purpleMask, greenMask, combinedMask);
        Core.bitwise_and(combinedMask, roiMask, combinedMask);

        // Find contours (blobs)
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(combinedMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Clear old centers
        detectedCenters.clear();

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > MIN_AREA) {
                Moments m = Imgproc.moments(contour);
                if (m.m00 != 0) {
                    int cx = (int)(m.m10 / m.m00);
                    int cy = (int)(m.m01 / m.m00);
                    if(cy>acceptPixelsAbove)
                    {
                        continue;
                    }
                    Point center = new Point(cx, cy);

                    String colorLabel = "Unknown";
                    if (purpleMask.get(cy, cx)[0] > 0) {
                        colorLabel = "P";
                    } else if (greenMask.get(cy, cx)[0] > 0) {
                        colorLabel = "G";
                    }

                    // Save center
                    detectedCenters.add(center);
                    Imgproc.circle(frame, center, 5, new Scalar(255, 255, 255), -1);
                    Imgproc.putText(frame, colorLabel, center, Imgproc.FONT_HERSHEY_SIMPLEX, 0.7, new Scalar(255, 255, 255), 2);


                    if (telemetry != null) {
                        telemetry.addData("Blob", "%s at (%d, %d)", colorLabel, cx, cy);
                    }
                }
            }
        }

        setBallsInRamp(detectedCenters.size());

        Mat masked = new Mat();
        Core.bitwise_and(frame, frame, masked, combinedMask);
        return masked;
    }


    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
