package org.firstinspires.ftc.teamcode.robot.subsystems.vision;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

public class VisionPipeline extends OpenCvPipeline
{

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
    public Mat processFrame(Mat input)
    {
        Imgproc.rectangle(
                input,
                new Point(
                        input.cols()/4,
                        input.rows()/4),
                new Point(
                        input.cols()*(3f/4f),
                        input.rows()*(3f/4f)),
                new Scalar(0, 255, 0), 4);


        return input;
    }

    @Override
    public void onViewportTapped()
    {

        viewportPaused = !viewportPaused;

        if(viewportPaused)
        {
            camera.pauseViewport();
        }
        else
        {
            camera.resumeViewport();
        }
    }
}
