package org.firstinspires.ftc.teamcode.robot.subsystems.vision.AprilTag;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;


public class AprilTagPipeline extends OpenCvPipeline
{
    public static double fx = 578.272;
    public static double fy = 578.272;
    public static double cx = 402.145;
    public static double cy = 221.506;

    public static double TAG_SIZE = 0.166;


    private long nativeApriltagPtr;
    private Mat grey = new Mat();
    private ArrayList<AprilTagDetection> detections = new ArrayList<>();

    private ArrayList<AprilTagDetection> detectionsUpdate = new ArrayList<>();

    private float decimation;
    private final Object detectionsUpdateSync = new Object();
    private boolean needToSetDecimation;
    private final Object decimationSync = new Object();

    Telemetry telemetry;

    public AprilTagPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(Mat frame)
    {
        nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
    }

    @Override
    public Mat processFrame(Mat input)
    {
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);

        synchronized (decimationSync)
        {
            if(needToSetDecimation)
            {
                AprilTagDetectorJNI.setApriltagDetectorDecimation(nativeApriltagPtr, decimation);
                needToSetDecimation = false;
            }
        }

        // Run AprilTag
        detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, grey, TAG_SIZE, fx, fy, cx, cy);

        synchronized (detectionsUpdateSync)
        {
            detectionsUpdate = detections;
        }

        for(AprilTagDetection detection : detections)
        {
            Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

            telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
            telemetry.addLine(String.format("Translation X: %.2f m", detection.pose.x));
            telemetry.addLine(String.format("Translation Y: %.2f m", detection.pose.y));
            telemetry.addLine(String.format("Translation Z: %.2f m", detection.pose.z));

            telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
            telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
            telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
        }

        telemetry.update();

        return input;
    }

    public ArrayList<AprilTagDetection> getLatestDetections()
    {
        return detections;
    }

    public ArrayList<AprilTagDetection> getDetectionsUpdate()
    {
        synchronized (detectionsUpdateSync)
        {
            ArrayList<AprilTagDetection> ret = detectionsUpdate;
            detectionsUpdate = null;
            return ret;
        }
    }
}