package org.firstinspires.ftc.teamcode.robot.subsystems.vision.Spindexer;

import android.graphics.Canvas;
import android.util.Log;

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

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class SpindexerPipeline implements VisionProcessor
{

    private static final String TAG = "SpindexerPipeline";

    private final Mat hsv = new Mat();
    private Mat output = new Mat();



    public static Scalar purpleLow1  = new Scalar(45.3, 77.9, 155.8);
    public static Scalar purpleHigh1 = new Scalar(121.8, 68, 255);
    public static Scalar purpleLow2  = new Scalar(117.6, 59.5, 97.8);
    public static Scalar purpleHigh2 = new Scalar(168.6, 255, 255);

    //    public static Scalar greenLow  = new Scalar(29.8, 89.3, 19.8);
    //    public static Scalar greenHigh = new Scalar(59.5, 144.5, 158.7);
    public static Scalar greenLow  = new Scalar(35, 40, 40);
    // Upper bound for green
    public static Scalar greenHigh = new Scalar(85, 255, 255);
    Mat purpleMask1 = new Mat();
    Mat purpleMask2 = new Mat();
    Mat purpleMask = new Mat();

    Mat greenMask = new Mat();

    Mat cameraMatrix;

    private Mat grey = new Mat();

    public static double fx = 578.272;
    public static double fy = 578.272;
    public static double cx = 402.145;
    public static double cy = 221.506;


    private OpenCvCamera camera;

    private float decimation;
    private boolean needToSetDecimation;
    private final Object decimationSync = new Object();

    private final List<Point> detectedCenters = new ArrayList<>();

    @Override
    public void init(int i, int i1, CameraCalibration cameraCalibration) {

    }

    private char[] spindexerPositions = {'N','N','N'};
    private char[] newPositions = {'N', 'N', 'N'};

    //yummy
    /**
     * @return order of the balls in the spindexer with top:0 right:1 left:2
     * G/P:colors, N:no ball detected
     */
    public char[] getBalls()
    {
        return spindexerPositions;
    }

    @Override
    public Object processFrame(Mat input, long l) {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, purpleLow1, purpleHigh1, purpleMask1);
        Core.inRange(hsv, purpleLow2, purpleHigh2, purpleMask2);
        Core.bitwise_or(purpleMask1, purpleMask2, purpleMask);
        Core.inRange(hsv, greenLow, greenHigh, greenMask);
        Mat combinedMask = new Mat();
        Core.bitwise_or(purpleMask, greenMask, combinedMask);
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(combinedMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        detectedCenters.clear();
        double minArea = 500.0;
        for(int i=0;i<3;i++)
        {
            newPositions[i] = 'N';
        }
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > minArea) {
                Moments m = Imgproc.moments(contour);

                if (m.m00 != 0) {
                    int cx = (int)(m.m10 / m.m00);
                    int cy = (int)(m.m01 / m.m00);
                    Point center = new Point(cx, cy);
                    String point = "";
                    int spindex = -1;
                    if(center.y < (double)input.height()/2)
                    {
                        point = "up";
                        spindex = 0;

                    }
                    else if(center.x < (double)input.width()/2)
                    {
                        point = "left";
                        spindex = 2;
                    }
                    else
                    {
                        spindex = 1;
                        point = "right";
                    }

                    String colorLabel = "Unknown";
                    if (purpleMask.get(cy, cx)[0] > 0) {
                        newPositions[spindex] = 'P';
                        colorLabel = "Purple" + point;
                    } else if (greenMask.get(cy, cx)[0] > 0) {
                        newPositions[spindex] = 'G';
                        colorLabel = "Green" + point;
                    }

                    detectedCenters.add(center);
                    Imgproc.circle(input, center, 5, new Scalar(255, 0, 0), -1);
                    Imgproc.putText(input, colorLabel, center, Imgproc.FONT_HERSHEY_SIMPLEX, 0.7, new Scalar(255, 255, 255), 2);

                    Log.i(TAG, String.format("Blob %s at (%d, %d)", colorLabel, cx, cy)); //i trust aadit is a genius
                }
            }
        }
        spindexerPositions = newPositions;
        Mat masked = new Mat();
        Core.bitwise_and(input, input, masked, combinedMask);
        return masked;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int i, int i1, float v, float v1, Object o) {

    }
}
