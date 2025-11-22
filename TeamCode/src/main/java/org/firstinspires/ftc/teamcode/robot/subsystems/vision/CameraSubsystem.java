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

//    private double turretCenterToRobotCenterUnitNeededX = 0; //this is perfectly centered
    public static double turretCenterToRobotCenterMeters = 0.2311;

    //robot dimemnnsions 16 inches by 15 inches

    public static double turretRadiusMeters = 0.13335;

    public static double cameraPhaseChangeAngleRadians = Math.toRadians(120);

    public static double tempTurretAngle = 0;

    public static Pose2d blueGlobalPose = new Pose2d(3000,3000,0.959931);

    public static Pose2d redGlobalPose = new Pose2d(3000,3000,-0.959931);

    public static Pose2d motifGlobalPose = new Pose2d(3000,3000,0);


    Telemetry telemetry;
    private AprilTagProcessor aTagProcessor;

    public enum GLYPH {
        GPP, PGP, PPG
    }

    public enum LiveViewSettings {OFF, FIELD}

    public Robot robot;
    public RobotHardware hardware;


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

    private Telemetry tele;

    public CameraSubsystem(Telemetry tele,Robot robot, RobotHardware hardware, LiveViewSettings liveViewSettings) {
        this.robot = robot;
        this.tele = tele;
        this.hardware = hardware;
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
        if(!detections.isEmpty())
        {
            tele.addData("seentagpos",getRobotCenterCoordinateToAprilTag());
            tele.addData("turretAngle",robot.shooter.turretAngle);
            tele.addData("yaw",detections.get(0).ftcPose.yaw);
            tele.addData("pitch",detections.get(0).ftcPose.pitch);
            tele.addData("roll",detections.get(0).ftcPose.roll);
        }
    }

    public boolean hasDetections() {
        return !detections.isEmpty();
    }

    public double getOrientationFromCameraRad(AprilTagDetection tag)
    {
        /*"global banking" is a value that is given per tag, as in if the robot were looking forward like this
       (-55deg)     (55deg)
        *  /     \
              ^
              |
              |
              |


        given this situation

      (global banked at -55)
        /(camera returns the tag being banked at -35)
            ^
            | (turret angle is facing 270 in code (hopefully)
   180 <----  (robot's global angle)

        clockkwise positiev

        globalBanking+seenAngle+turret=robots global
        -55-35+270=(-90)+270=180
        this should hopefully work (can't wait for it to not)
        */
        double heading;
        switch(VisionConstants.APRILTAG.tagMap.get(tag.id))
        {
            case "BLUESCORE":
                heading = blueGlobalPose.heading;
                break;
//            case "GPP":
//                break;
//            case "PGP":
//                break;
//            case "PPG":
//                break;
            case "REDSCORE":
                heading = redGlobalPose.heading;
                break;
            default:
                heading = motifGlobalPose.heading;
                break;
        }
        return heading+tag.ftcPose.range+tempTurretAngle;

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
        return ((tag.ftcPose.x/39.37) + turretRadiusMeters*Math.cos(tempTurretAngle + cameraPhaseChangeAngleRadians));
    }

    private double getCoordinateComponentY(AprilTagDetection tag) {
        return (tag.ftcPose.y/39.37 + turretRadiusMeters*Math.sin(tempTurretAngle + cameraPhaseChangeAngleRadians) + turretCenterToRobotCenterMeters);
    }

    public Pose2d getRobotCenterCoordinateToAprilTag() {
        AprilTagDetection tag = detections.get(0);
        return new Pose2d(getCoordinateComponentX(tag), getCoordinateComponentY(tag),getOrientationFromCameraRad(tag));
    }
}