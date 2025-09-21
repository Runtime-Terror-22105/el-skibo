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

    Scalar purpleLow1  = new Scalar(136, 70, 124);
    Scalar purpleHigh1 = new Scalar(179, 254, 254);
    Scalar purpleLow2  = new Scalar(0, 70, 124);
    Scalar purpleHigh2 = new Scalar(148, 95, 254);

    Scalar greenLow  = new Scalar(82, 255, 109);
    Scalar greenHigh = new Scalar(83, 254, 245);
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
        Imgproc.putText(
                masked,
                Integer.toString(detections.size()),
                new Point(100,300),
                Imgproc.FONT_HERSHEY_SIMPLEX,
                10,
                new Scalar(255, 0, 0),
                2
        );

        for (AprilTagDetection tag : detections) {
            Point[] corners = tag.corners;
            for (int i = 0; i < 4; i++) {
                Imgproc.line(
                        masked,
                        corners[i],
                        corners[(i + 1) % 4],
                        new Scalar(0, 255, 0), 2
                );
            }
        }

        return masked;
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
