package org.firstinspires.ftc.teamcode.robot.subsystems.vision;

import android.util.Log;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.math.Coordinate;
import org.firstinspires.ftc.teamcode.pedroPathing.FtcDashDrawing;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

@Config
public class CameraSubsystem extends SubsystemBase {
    public static boolean disableRelocalization = false;
    public static Coordinate cameraToTurretCenterOffset = new Coordinate(4.7, 2.2);
    public static Coordinate turretToRobotCenterOffset = new Coordinate(-2.2, 0);

    public static double CONVERGENCE_RATE = 0.1;
    public static double VELOCITY_THRESHOLD = 5.0; // inches per second

    public static long EXPOSURE_MICROSECONDS = 200;
    public static int GAIN = 255;

    private AprilTagProcessorDash aTagProcessor;

    private boolean shouldScanForGlyphs = false;

    public enum GLYPH {
        GPP, PGP, PPG
    }

    public enum LiveViewSettings {OFF, FIELD}

    public Robot robot;
    public RobotHardware hardware;

    public GLYPH gameGlyph;
    private boolean decodedGlyph = true; //when the movie uses the title of the movie

    private final VisionPortal.Builder vPortalBuilder = new VisionPortal.Builder();
    public final VisionPortal vPortalField;

    private ArrayList<AprilTagDetection> detections;

    private Pose debugLastDetection = null; // for debug only
    private long debugDetectionTime = 0; // for debug only

    public CameraSubsystem() {
        this.vPortalField = null;
        this.shouldScanForGlyphs = true;
    }

    public CameraSubsystem(Robot robot, RobotHardware hardware, LiveViewSettings liveViewSettings) {
        this.robot = robot;
        this.hardware = hardware;
        this.detections = new ArrayList<>();
        this.aTagProcessor = new AprilTagProcessorDash(createAprilTagProcessor());

        VisionPortal.Builder vPortalFieldBuilder = new VisionPortal.Builder()
                .setCamera(hardware.fieldCamera)
//                .setCameraResolution(new Size(320, 240))
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

        // Reference for Logitech C270:
//        public void setDefaultExposure() {
//            this.camera.getExposureControl().setMode(ExposureControl.Mode.AperturePriority);
//            this.camera.getGainControl().setGain(64);
//        }
//
//        public void setLowExposure() {
//            this.camera.getExposureControl().setMode(ExposureControl.Mode.Manual);
//            this.camera.getExposureControl().setExposure(100, TimeUnit.MICROSECONDS);
//            this.camera.getGainControl().setGain(255);
//        }
        ExposureControl exposure = vPortalField.getCameraControl(ExposureControl.class);
        GainControl gain = vPortalField.getCameraControl(GainControl.class);
        exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure(EXPOSURE_MICROSECONDS, TimeUnit.MICROSECONDS);
        gain.setGain(GAIN);

        FtcDashboard.getInstance().startCameraStream(vPortalField, 0);
        this.shouldScanForGlyphs = true;
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
                .setLensIntrinsics(910.121, 910.121, 648.374, 394.354)
                .build();
    }

    public void stopScanningForGlyphs() {
        this.shouldScanForGlyphs = false;
    }

    public void startScanningForGlyphs() {
        this.shouldScanForGlyphs = true;
    }

    public GLYPH getGlyph() {
        if (decodedGlyph) {
            return gameGlyph;
        }
        return null;
    }

    public char[] getGlyphCharArray() {
        if (decodedGlyph) {
            switch(gameGlyph)
            {
                case GPP:
                    return new char[]{'G','P','P'};

                case PGP:
                    return new char[]{'P','G','P'};

                case PPG:
                    return new char[]{'P','P','G'};
            }
        }
        return null;
    }

    public void setGlyph(GLYPH glyph) {
        decodedGlyph = true;
        gameGlyph = glyph;
        Log.i("CameraSubsystem", "Found glyph " + gameGlyph);
    }

    @Override
    public void periodic() {
        if (vPortalField == null) return;

        this.detections = aTagProcessor.getDetections();
        //should only ever be the blue or red goal which is 20 and 24 respectively
        AprilTagDetection localizationTag = null;

        for (AprilTagDetection tag : detections) {
            if (tag.id >= 21 && tag.id <= 23) {
                robot.telemetry.addData("seenButUnusedGlyph", GLYPH.valueOf(VisionConstants.APRILTAG.tagMap.get(tag.id)));
                if (!decodedGlyph && shouldScanForGlyphs)
                    setGlyph(GLYPH.valueOf(VisionConstants.APRILTAG.tagMap.get(tag.id)));
            } else {
                localizationTag = tag;
            }
        }
        robot.telemetry.addData("Glyph", gameGlyph);
        robot.telemetry.addData("Velocity Magnitude", robot.follower.getVelocity().getMagnitude());
        robot.telemetry.addData("Localization Tag", localizationTag);
        if (localizationTag != null && localizationTag.robotPose != null
            && robot.follower.getVelocity().getMagnitude() < VELOCITY_THRESHOLD) {
            handleLocalizationDetection(localizationTag);
        }

        if (debugLastDetection != null) {
            // Fade color from red to black as detection gets older
            int ageMs = (int) (System.currentTimeMillis() - debugDetectionTime);
            int red = Math.max(0, 255 - ageMs / 5);
            FtcDashDrawing.drawRobot(debugLastDetection != null ? debugLastDetection : new Pose(0, 0, 0), String.format("#%02X0000", red));
        }
    }

    private void handleLocalizationDetection(AprilTagDetection tag) {
        Pose cameraFieldPose = new Pose(72 + tag.robotPose.getPosition().y, 72 - tag.robotPose.getPosition().x, tag.robotPose.getOrientation().getYaw(AngleUnit.RADIANS) + Math.PI);
//        FtcDashDrawing.drawDot(cameraFieldPose, "#0000FF");

        double turretAngle = robot.shooter.getGoalTurretYaw();
        double cameraRobotHeading = cameraFieldPose.getHeading() - turretAngle;
        double pinpointRobotHeading = robot.follower.poseTracker.getPose().getHeading();

        // If the Pinpoint IMU heading is extremely off from the calculated angle, this means we've
        // likely inited the IMU in the wrong position. So, we will do a "full localization reset"
        // to correct it.
//        boolean isFullReset = MathFunctions.getSmallestAngleDifference(cameraRobotHeading, pinpointRobotHeading) < Math.toRadians(FULL_RESET_THRESHOLD_ANGLE);
        double robotHeading;
        double convergenceRate;
        robotHeading = pinpointRobotHeading;
        convergenceRate = CONVERGENCE_RATE;

        Pose turretVector = new Pose(cameraToTurretCenterOffset.x, cameraToTurretCenterOffset.y, 0).rotate(cameraFieldPose.getHeading(), false);
        Pose turretCenter = cameraFieldPose.minus(turretVector);
//        FtcDashDrawing.drawDot(turretCenter, "#FF00FF");
        Pose robotVector = new Pose(turretToRobotCenterOffset.x, turretToRobotCenterOffset.y, 0).rotate(robotHeading, false);
        Pose robotCenter = turretCenter.minus(robotVector);

        Pose robotPose = new Pose(robotCenter.getX(), robotCenter.getY(), robotHeading);

        debugLastDetection = robotPose;
        debugDetectionTime = System.currentTimeMillis();

        // Apply exponential convergence
        if (!disableRelocalization) {
            Pose currentPose = robot.follower.poseTracker.getPose();
            Pose convergedPose = new Pose(
                    currentPose.getX() + convergenceRate * (robotPose.getX() - currentPose.getX()),
                    currentPose.getY() + convergenceRate * (robotPose.getY() - currentPose.getY()),
                    currentPose.getHeading() + convergenceRate * (robotPose.getHeading() - currentPose.getHeading())
            );
            robot.follower.poseTracker.setCurrentPoseWithOffset(convergedPose);
        }
    }
}