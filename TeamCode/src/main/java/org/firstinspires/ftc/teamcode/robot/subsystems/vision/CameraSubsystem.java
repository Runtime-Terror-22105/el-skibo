package org.firstinspires.ftc.teamcode.robot.subsystems.vision;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.ShooterSubsystem;
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
public class CameraSubsystem extends SubsystemBase {

    //TODO: turrentCenterDists, turretRadiusvalue, cameraphasechange all values needed, and the pose2d in
    //TODO: getRobotCenterCoordinateToAprilTag heading is needed

    private OpenCvCamera aprilTagCamera;
    private OpenCvCamera spindexerCamera;

    private double turretCenterToRobotCenterUnitNeededX = -1;
    private double turretCenterToRobotCenterUnitNeededY = 1;

    //robot dimemnnsions 16 inches by 15 inches

    private final double turretRadiusMeters = 0.13335;

    private final double cameraPhaseChangeAngleRadians = Math.toRadians(120);


    Telemetry telemetry;
    private AprilTagProcessor aTagProcessor;

    public enum GLYPH {
        GPP, PGP, PPG
    }

    public enum LiveViewSettings {OFF, FIELD}

    public Robot robot = new Robot();


    public GLYPH gameGlyph;
    private boolean decodedGlyph = false; //when the movie uses the title of the movie

    public GLYPH getGlyph() {
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

        VisionPortal.Builder vPortalFieldBuilder = new VisionPortal.Builder()
                .setCamera(hardware.fieldCamera)
                .setCameraResolution(new Size(320, 240))
                .setCameraResolution(new Size(1280, 800))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .addProcessor(this.aTagProcessor);

        switch (liveViewSettings) {
            case FIELD:
                vPortalFieldBuilder.enableLiveView(true);
//                        .setLiveViewContainerId(hardware.cameraMonitorViewId);
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

        for (AprilTagDetection tag : detections) {
            if (tag.id >= 21 && tag.id <= 23 && !decodedGlyph) {
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

    //this gives the actual coords from the robot center to the april tag which makes the robot center 0,0 relative
    //to the outputed value

    private double getCoordinateComponentX(AprilTagDetection tag) {
        return ((tag.ftcPose.x/39.37) + turretRadiusMeters*Math.cos(robot.shooter.turretAngle + cameraPhaseChangeAngleRadians) + turretCenterToRobotCenterUnitNeededX);
    }

    private double getCoordinateComponentY(AprilTagDetection tag) {
        return (tag.ftcPose.y/39.37 + turretRadiusMeters*Math.sin(robot.shooter.turretAngle + cameraPhaseChangeAngleRadians) + turretCenterToRobotCenterUnitNeededY);
    }

    public Pose2d getRobotCenterCoordinateToAprilTag() {
        AprilTagDetection tag = detections.get(0);
        return new Pose2d(getCoordinateComponentX(tag), getCoordinateComponentY(tag), 0.0);
    }
}