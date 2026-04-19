package org.firstinspires.ftc.teamcode.robot.subsystems.vision;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
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

    Mat purpleMask = new Mat();

    Mat greenMask = new Mat();

    private Mat roiMask = new Mat();
    public static double minArea = 50; // tune this for noise filtering

    public static Point[] ROI_POINTS = {
            new Point(40,100),
            new Point(40,140),
            new Point(50,140),
            new Point(50,180),
            new Point(280,180),
            new Point(250,70),
    };

    private final List<Point> detectedCenters = new ArrayList<>();
    Telemetry telemetry;

    public int ballsSeen = 0;

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
//        Imgproc.GaussianBlur(hsv,hsv,new Size(1,1),0);
//        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5,5));
//        Imgproc.morphologyEx(purpleMask, purpleMask, Imgproc.MORPH_OPEN, kernel);
//        Imgproc.morphologyEx(greenMask, greenMask, Imgproc.MORPH_OPEN, kernel);

        Core.inRange(hsv, ColorRange.RampAltValues.purpleLow, ColorRange.RampAltValues.purpleHigh, purpleMask);
        Core.inRange(hsv, ColorRange.RampAltValues.greenLow, ColorRange.RampAltValues.greenHigh, greenMask);

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


//        Mat combinedMask = new Mat();
//        Core.bitwise_or(purpleMask, greenMask, combinedMask);
//        Core.bitwise_and(combinedMask, roiMask, combinedMask);
//        List<MatOfPoint> contours = new ArrayList<>();
//        Mat hierarchy = new Mat();
//        Imgproc.findContours(combinedMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
//        detectedCenters.clear();

        for (MatOfPoint contour : greenContours) {
            double area = Imgproc.contourArea(contour);
            if (area > minArea) {
                Moments m = Imgproc.moments(contour);
                if (m.m00 != 0) {
                    int cx = (int)(m.m10 / m.m00);
                    int cy = (int)(m.m01 / m.m00);
                    Point center = new Point(cx, cy);

                    detectedCenters.add(center);

                    Imgproc.circle(frame, center, 5, new Scalar(0, 255, 0), -1);
                    Imgproc.putText(frame, "G", center,
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

                    Imgproc.circle(frame, center, 5, new Scalar(255, 0, 255), -1);
                    Imgproc.putText(frame, "P", center,
                            Imgproc.FONT_HERSHEY_SIMPLEX, 0.7,
                            new Scalar(255, 0, 255), 2);

                    if (telemetry != null) {
                        telemetry.addData("Purple Blob", "(%d, %d)", cx, cy);
                    }
                }
            }
        }

        this.ballsSeen = detectedCenters.size();

        Mat masked = new Mat();
        Core.bitwise_and(frame, frame, masked, combinedMask);
        return masked;
    }

    public int getBalls()
    {
        return this.ballsSeen;
    }


    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
