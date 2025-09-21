package org.firstinspires.ftc.teamcode.robot.subsystems.vision.eocv;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class VisionPipeline extends OpenCvPipeline
{

    public VisionPipeline() {

    }

    private final Mat hsv = new Mat();
    private Mat output = new Mat();

    public static Scalar purpleLow1  = new Scalar(45.3, 77.9, 155.8);
    public static Scalar purpleHigh1 = new Scalar(121.8, 68, 255);
    public static Scalar purpleLow2  = new Scalar(117.6, 59.5, 97.8);
    public static Scalar purpleHigh2 = new Scalar(168.6, 255, 255);

    public static Scalar greenLow  = new Scalar(29.8, 89.3, 19.8);
    public static Scalar greenHigh = new Scalar(59.5, 144.5, 158.7);
    Mat purpleMask1 = new Mat();
    Mat purpleMask2 = new Mat();
    Mat purpleMask = new Mat();

    Mat greenMask = new Mat();



    private AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagID(true)
            .setDrawTagOutline(true)
            .setSuppressCalibrationWarnings(false)
            .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
            .setNumThreads(3)
            .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            .build();

    private OpenCvCamera camera;

    public VisionPipeline(OpenCvCamera camera)
    {
        this.vPortalBuilder.addProcessor(this.tagProcessor);
        this.camera = camera;
    }
    boolean viewportPaused;
    private final VisionPortal.Builder vPortalBuilder = new VisionPortal.Builder();

    public ArrayList<AprilTagDetection> getSeenTags()
    {
        return tagProcessor.getDetections();
    }


    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, purpleLow1, purpleHigh1, purpleMask1);
        Core.inRange(hsv, purpleLow2, purpleHigh2, purpleMask2);
        Core.bitwise_or(purpleMask1, purpleMask2, purpleMask);
        Core.inRange(hsv, greenLow, greenHigh, greenMask);

        Mat combinedMask = new Mat();
        Core.bitwise_or(purpleMask, greenMask, combinedMask);

        Mat masked = new Mat();
        Core.bitwise_and(input, input, masked, combinedMask);
        ArrayList<AprilTagDetection> detections = tagProcessor.getDetections();
//        Imgproc.putText(
//                masked,
//                Integer.toString(detections.size()),
//                new Point(100,300),
//                Imgproc.FONT_HERSHEY_SIMPLEX,
//                10,
//                new Scalar(255, 0, 0),
//                2
//        );
//
//        for (AprilTagDetection tag : detections) {
//            Point[] corners = tag.corners;
//            for (int i = 0; i < 4; i++) {
//                Imgproc.line(
//                        masked,
//                        corners[i],
//                        corners[(i + 1) % 4],
//                        new Scalar(0, 255, 0), 2
//                );
//            }
//        }
//
        return masked;
//        ArrayList<AprilTagDetection> detections = tagProcessor.getDetections();
//
//        for (AprilTagDetection tag : detections) {
//            Point[] corners = tag.corners;
//            for (int i = 0; i < 4; i++) {
//                Imgproc.line(input, corners[i], corners[(i + 1) % 4], new Scalar(0, 255, 0), 2);
//            }
//            Imgproc.putText(input, "ID:" + tag.id, corners[0], Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(0,0,255), 2);
//        }
//
//        return input;
    }


    @Override
    public void onViewportTapped()
    {
//
//        viewportPaused = !viewportPaused;
//
//        if(viewportPaused)
//        {
//            camera.pauseViewport();
//        }
//        else
//        {
//            camera.resumeViewport();
//        }
    }
}
