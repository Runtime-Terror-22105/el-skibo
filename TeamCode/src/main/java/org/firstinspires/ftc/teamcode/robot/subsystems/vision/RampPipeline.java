package org.firstinspires.ftc.teamcode.robot.subsystems.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.imgproc.Moments;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class RampPipeline extends OpenCvPipeline
{



    private final Mat hsv = new Mat();
    private Mat output = new Mat();

    public static double acceptPixelsAbove = 820;

    public static Scalar purpleLow1  = new Scalar(45.3, 77.9, 155.8);
    public static Scalar purpleHigh1 = new Scalar(121.8, 68, 255);
    public static Scalar purpleLow2  = new Scalar(120.4, 76.5, 48.2);
    public static Scalar purpleHigh2 = new Scalar(165.8, 255, 255);

    //    public static Scalar greenLow  = new Scalar(29.8, 89.3, 19.8);
//    public static Scalar greenHigh = new Scalar(59.5, 144.5, 158.7);
    public static Scalar greenLow  = new Scalar(46, 0, 0);
    // Upper bound for green
    public static Scalar greenHigh = new Scalar(102, 255, 250);
    Mat purpleMask1 = new Mat();
    Mat purpleMask2 = new Mat();
    Mat purpleMask = new Mat();

    Mat greenMask = new Mat();

    public int ballsInRamp = 0;

    public int getBallsInRamp()
    {
        return this.ballsInRamp;
    }

    private void setBallsInRamp(int amount)
    {
        this.ballsInRamp = amount;
    }

    Mat cameraMatrix;

    private Mat grey = new Mat();

    public static double fx = 578.272;
    public static double fy = 578.272;
    public static double cx = 402.145;
    public static double cy = 221.506;
//    private char[] rampBalls = {'N','N','N','N','N','N','N','N','N'};


    private OpenCvCamera camera;

    private float decimation;
    private boolean needToSetDecimation;
    private final Object decimationSync = new Object();

    private final List<Point> detectedCenters = new ArrayList<>();
    Telemetry telemetry;

    public RampPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    private final VisionPortal.Builder vPortalBuilder = new VisionPortal.Builder();

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, purpleLow1, purpleHigh1, purpleMask1);
        Core.inRange(hsv, purpleLow2, purpleHigh2, purpleMask2);
        Core.bitwise_or(purpleMask1, purpleMask2, purpleMask);
        Core.inRange(hsv, greenLow, greenHigh, greenMask);

        Mat combinedMask = new Mat();
        Core.bitwise_or(purpleMask, greenMask, combinedMask);

        // Find contours (blobs)
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(combinedMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Clear old centers
        detectedCenters.clear();

        double minArea = 500.0; // tune this for noise filtering

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > minArea) {
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
                    Imgproc.circle(input, center, 5, new Scalar(255, 255, 255), -1);
                    Imgproc.putText(input, colorLabel, center, Imgproc.FONT_HERSHEY_SIMPLEX, 0.7, new Scalar(255, 255, 255), 2);


                    if (telemetry != null) {
                        telemetry.addData("Blob", "%s at (%d, %d)", colorLabel, cx, cy);
                    }
                }
            }
        }

        setBallsInRamp(detectedCenters.size());

//        System.out.println(detectedCenters);

//        blobs : [{1239.0, 818.0}, {1134.0, 788.0}, {1028.0, 758.0}, {934.0, 730.0}, {840.0, 706.0}, {737.0, 677.0}, {619.0, 644.0}, {526.0, 623.0}, {411.0, 585.0}]

        if (telemetry != null) {
            telemetry.addData("blobs", detectedCenters);
        }
//        telemetry.addData("blobs", detectedCenters);
        Mat masked = new Mat();
        Core.bitwise_and(input, input, masked, combinedMask);
//        Imgproc.putText(input, "the", new Point(150,acceptPixelsAbove), Imgproc.FONT_HERSHEY_SIMPLEX, 0.7, new Scalar(255, 0,0), 2);
        return masked;
    }
}