package org.firstinspires.ftc.teamcode.robot.subsystems.vision;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.subsystems.vision.Spindexer.SpindexerPipeline;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@TeleOp
public class CameraSubsystem extends SubsystemBase
{
    private OpenCvWebcam webcam;

    private SpindexerPipeline pipeline = new SpindexerPipeline(webcam);

    private ArrayList<AprilTagDetection> detections;

    public Pose2d currentPosition= new Pose2d(0,0,0);
    public enum GLYPH {
        GPP,PGP,PPG
    }

    private GLYPH gameGlyph;
    private boolean decodedGlyph = false; //when the movie uses the title of the movie

    public GLYPH getGlyph()
    {
        return gameGlyph;
    }

    public boolean isDetecting(){ // if camera is detecting april tag
        return this.detections.size()>0;
    }

    public Pose2d getPositionCamera(){return this.currentPosition;}

    @Override
    public void periodic() {

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
            detections = pipeline.getSeenTags();
            if(detections.size() != 0)
            {
                for(AprilTagDetection tag : detections)
                {
                    if(tag.id >= 21 && tag.id <= 23 && !decodedGlyph)
                    {
                        gameGlyph = GLYPH.valueOf(VisionConstants.APRILTAG.tagMap.get(tag.id));
                        //decodedGlyph = true;
                    }
                    telemetry.addData("Tag ID",tag.id);
                    telemetry.addData("Nickname Tag", VisionConstants.APRILTAG.tagMap.get(tag.id));
                    //technically unsafe if the tag is somehow falsely read or someone in the audience wears an apriltag shirt
                    telemetry.addData("Pose X", tag.robotPose.getPosition().x); //i have NO idea what this returns ehl em ay oh
                    telemetry.addData("Pose Y", tag.robotPose.getPosition().y);
                    telemetry.addData("Pose Z", tag.robotPose.getPosition().z);
                    currentPosition= new Pose2d(tag.robotPose.getPosition().x,tag.robotPose.getPosition().y,tag.robotPose.getPosition().z);
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
        }
    }