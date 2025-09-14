package org.firstinspires.ftc.teamcode.robot.subsystems.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@TeleOp
public class CameraSubsystem extends LinearOpMode
{
    private OpenCvWebcam webcam;

    private VisionPipeline pipeline = new VisionPipeline(webcam);

    private ArrayList<AprilTagDetection> detections;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(5000);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        waitForStart();

        while (opModeIsActive()) {
            detections = pipeline.getSeenTags();
            if(detections.size() != 0)
            {
                for(AprilTagDetection tag : detections)
                {
                    telemetry.addData("Tag ID",tag.id);
                    telemetry.addData("Nickname Tag", VisionConstants.APRILTAG.tagMap.get(tag.id));
                    //technically unsafe if the tag is somehow falsely read or someone in the audience wears an apriltag shirt
                    telemetry.addData("Pose X", tag.robotPose.getPosition().x); //i have NO idea what this returns ehl em ay oh
                    telemetry.addData("Pose Y", tag.robotPose.getPosition().y);
                    telemetry.addData("Pose Z", tag.robotPose.getPosition().z);
                }
            }
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", webcam.getFps());
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());


            telemetry.update();

            if (gamepad1.a) {
                webcam.stopStreaming();
            }
            sleep(100);
        }
    }

}