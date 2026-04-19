package org.firstinspires.ftc.teamcode.robot.subsystems.vision.ramptest;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class EOCV_Ramp extends OpenCvPipeline
{
    private final Mat hsv = new Mat();
    public static Scalar purpleLow  = new Scalar(147.3, 83.6, 45.3);
    public static Scalar purpleHigh = new Scalar(255, 255, 255);
    public static Scalar greenLow  = new Scalar(46.8, 109.1, 0);
    public static Scalar greenHigh = new Scalar(87.8, 255, 184.2);

    public static double minArea = 500.0; // tune this for noise filtering
    Mat purpleMask = new Mat();

    Mat greenMask = new Mat();

    public int ballsInRamp = 0;

    private void setBallsInRamp(int amount)
    {
        this.ballsInRamp = amount;
    }
    private final List<Point> detectedCenters = new ArrayList<>();
    Telemetry telemetry;

    public EOCV_Ramp() {

    }
    private final VisionPortal.Builder vPortalBuilder = new VisionPortal.Builder();

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, purpleLow, purpleHigh, purpleMask);
        Core.inRange(hsv, greenLow, greenHigh, greenMask);

        Mat combinedMask = new Mat();
        Core.bitwise_or(purpleMask, greenMask, combinedMask);

        // Find contours (blobs)
        List<MatOfPoint> greenContours = new ArrayList<>();
        List<MatOfPoint> purpleContours = new ArrayList<>();
        Mat hierarchy = new Mat();

        Imgproc.findContours(greenMask, greenContours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(purpleMask, purpleContours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Clear old centers
        detectedCenters.clear();

//        for (MatOfPoint contour : greenContours) {
//            double area = Imgproc.contourArea(contour);
//            if (area > minArea) {
//                Moments m = Imgproc.moments(contour);
//                if (m.m00 != 0) {
//                    int cx = (int)(m.m10 / m.m00);
//                    int cy = (int)(m.m01 / m.m00);
//                    Point center = new Point(cx, cy);
//
//                    String colorLabel = "Unknown";
//                    if (purpleMask.get(cy, cx)[0] > 0) {
//                        colorLabel = "P";
//                    } else if (greenMask.get(cy, cx)[0] > 0) {
//                        colorLabel = "G";
//                    }
//
//                    // Save center
//                    detectedCenters.add(center);
//                    Imgproc.circle(input, center, 5, new Scalar(255, 255, 255), -1);
//                    Imgproc.putText(input, colorLabel, center, Imgproc.FONT_HERSHEY_SIMPLEX, 0.7, new Scalar(255, 255, 255), 2);
//
//
//                    if (telemetry != null) {
//                        telemetry.addData("Blob", "%s at (%d, %d)", colorLabel, cx, cy);
//                    }
//                }
//            }
//        }

        for (MatOfPoint contour : greenContours) {
            double area = Imgproc.contourArea(contour);
            if (area > minArea) {
                Moments m = Imgproc.moments(contour);
                if (m.m00 != 0) {
                    int cx = (int)(m.m10 / m.m00);
                    int cy = (int)(m.m01 / m.m00);
                    Point center = new Point(cx, cy);

                    detectedCenters.add(center);

                    Imgproc.circle(input, center, 5, new Scalar(0, 255, 0), -1);
                    Imgproc.putText(input, "G", center,
                            Imgproc.FONT_HERSHEY_SIMPLEX, 0.7,
                            new Scalar(0, 255, 0), 2);

                    if (telemetry != null) {
                        telemetry.addData("Green Blob", "(%d, %d)", cx, cy);
                    }
                }
            }
        }

        for (MatOfPoint contour : purpleContours) {
            double area = Imgproc.contourArea(contour);
            if (area > minArea) {
                Moments m = Imgproc.moments(contour);
                if (m.m00 != 0) {
                    int cx = (int)(m.m10 / m.m00);
                    int cy = (int)(m.m01 / m.m00);
                    Point center = new Point(cx, cy);

                    detectedCenters.add(center);

                    Imgproc.circle(input, center, 5, new Scalar(255, 0, 255), -1);
                    Imgproc.putText(input, "P", center,
                            Imgproc.FONT_HERSHEY_SIMPLEX, 0.7,
                            new Scalar(255, 0, 255), 2);

                    if (telemetry != null) {
                        telemetry.addData("Purple Blob", "(%d, %d)", cx, cy);
                    }
                }
            }
        }

        setBallsInRamp(detectedCenters.size());

        Mat masked = new Mat();
        Core.bitwise_and(input, input, masked, combinedMask);
        return masked;
    }
}
