package org.firstinspires.ftc.teamcode.robot.subsystems.vision;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.ArrayList;

@Config
public class CameraSubsystem extends SubsystemBase
{
    private OpenCvCamera aprilTagCamera;
    private OpenCvCamera spindexerCamera;
    

    Telemetry telemetry;
    private AprilTagProcessor aTagProcessor;
    public enum GLYPH {
        GPP,PGP,PPG
    }

    public enum LiveViewSettings { OFF, FIELD }


    public GLYPH gameGlyph;
    private boolean decodedGlyph = false; //when the movie uses the title of the movie

    public GLYPH getGlyph()
    {
        return gameGlyph;
    }

    /**
     * @return order of the balls in the spindexer with top:0 right:1 left:2
     * G/P:colors, N:no ball detected
     */
//    public char[] getBalls()
//    {
//        return spindexerPipeline.getBalls();
//    }

    private final VisionPortal.Builder vPortalBuilder = new VisionPortal.Builder();
    public final VisionPortal vPortalField;

    private ArrayList<AprilTagDetection> detections;

    public CameraSubsystem(RobotHardware hardware, LiveViewSettings liveViewSettings) {
        this.detections = new ArrayList<>();
        this.aTagProcessor = createAprilTagProcessor();
//        this.spindexerPipeline = new SpindexerPipeline(telemetry);

        VisionPortal.Builder vPortalFieldBuilder = new VisionPortal.Builder()
                .setCamera(hardware.fieldCamera)
                .setCameraResolution(new Size(320, 240))
//                .addProcessor(this.spindexerPipeline) //sad emoji
                .addProcessor(this.aTagProcessor);

//        VisionPortal.Builder vPortalSpindexerBuilder = new VisionPortal.Builder()
//                .setCamera(hardware.fieldCamera)
//                .setCameraResolution(new Size(320, 240))
//                .addProcessor(this.spindexerPipeline);


        switch (liveViewSettings) {
            case FIELD:
                vPortalFieldBuilder.enableLiveView(true);
                // note: to have this appear in dashboard, you need to have the pipeline implement CameraStreamSource
                // see last year's code for reference, I'm too lazy to do this rn
//                FtcDashboard.getInstance().startCameraStream(atagPipeline, 0);
                break;
        }

        vPortalField = vPortalFieldBuilder.build();
//        vPortalSpindexer = vPortalSpindexerBuilder.build();
    }

    private AprilTagProcessor createAprilTagProcessor() {
        return new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setSuppressCalibrationWarnings(false)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
//                    .setLensIntrinsics() // TODO: placeholder to remind us to calibrate the camera
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS) // TODO: Placeholder
                .setNumThreads(3) // TODO: the default is 3 but maybe we can change
                .build();
    }

    @Override
    public void periodic() {
        this.detections = aTagProcessor.getDetections();

        for(AprilTagDetection tag : detections)
        {
            if(tag.id >= 21 && tag.id <= 23 && !decodedGlyph)
            {
                gameGlyph = GLYPH.valueOf(VisionConstants.APRILTAG.tagMap.get(tag.id));
                decodedGlyph = true;
            }
        }
    }

    public boolean hasDetections() {
        return !detections.isEmpty();
    }

    public Pose2d getPositionCamera()
    {

        if (detections.isEmpty()) {
            return null;
        }

        // todo: choose only one apriltag to use
        AprilTagDetection tag = detections.get(0);
        return new Pose2d(tag.robotPose.getPosition().x-VisionConstants.APRILTAG.cameraOffset.x,tag.robotPose.getPosition().y-VisionConstants.APRILTAG.cameraOffset.y,tag.robotPose.getPosition().z-VisionConstants.APRILTAG.cameraOffset.z);
    }
}