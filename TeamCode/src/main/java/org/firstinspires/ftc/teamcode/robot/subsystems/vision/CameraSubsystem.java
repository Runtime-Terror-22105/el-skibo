package org.firstinspires.ftc.teamcode.robot.subsystems.vision;

import android.graphics.Color;
import android.util.Log;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.SortOrder;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

@Config
public class CameraSubsystem extends SubsystemBase {

    private final Robot robot;
    private final RobotHardware hardware;

    public static String TAG = "CameraSubsystem";

    // =======================
    // portal
    // ======================
    private VisionPortal frontPortal;
    private VisionPortal backPortal;

    private AprilTagProcessor tagProcessor;
    private final BallDetectionPipeline ballPipeline;

    private final RampPipeline rampPipeline;

    private ArrayList<AprilTagDetection> detections = new ArrayList<>();

    private final int[] visionPortalIDs;

    // =======================
    // state
    // =======================
    private boolean scanForGlyphs = false;
    private boolean relocalizeSucceeded = false;
    public boolean relocalizationEnabled = false;

    public GLYPH gameGlyph;
    public GLYPH baseGlyph;
    public FRONT_CV_MODE CVMode = FRONT_CV_MODE.NONE;

    private int ballsSeen = 0;
    private final ElapsedTime relocalizeTimer = new ElapsedTime();

    // =======================
    // debug stuff
    // =======================
    private Pose debugLastDetection = null;
    private long debugDetectionTime = 0;

    // =======================
    // const
    // =======================
    public static double MIN_CONTOUR_AREA = 300;
    public static double MAX_CONTOUR_AREA = 100000;

    public static int RELOCALIZE_WINDOW_MS = 200;
    public static double CAMERA_OFFSET_IN = -8;

    public static double VELOCITY_THRESHOLD = 5.0;

    public static long EXPOSURE_US = 200;
    public static int GAIN = 255;

    public enum GLYPH {
        PPG(BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN),
        PGP(BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE),
        GPP(BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE);

        public final BallColor[] colors;

        public GLYPH rotate(int offset) {
            GLYPH[] vals = GLYPH.values();
            int newIndex = (this.ordinal() + offset) % vals.length;

            if (newIndex < 0) newIndex += vals.length;

            return vals[newIndex];
        }
        GLYPH(BallColor... colors) {
            this.colors = colors;
        }
    }

    public enum FRONT_CV_MODE
    {
        RAMP,FAR,NONE
    }

    // =======================
    // init
    // =======================
    public CameraSubsystem(Robot robot, RobotHardware hardware) {
        this.robot = robot;
        this.hardware = hardware;

        this.tagProcessor = createAprilTagProcessor();
        this.ballPipeline = createBallPipeline();
        this.rampPipeline = createRampPipeline();

        this.visionPortalIDs =  VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);

        initCameras();
    }

    private void initCameras() {
        if (hardware.frontCamera != null) {
            Log.i("CameraSubsytem", "front camera built" );
            frontPortal = new VisionPortal.Builder()
                    .setCamera(hardware.frontCamera)
                    .setCameraResolution(new Size(320, 240))
                    .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                    .setLiveViewContainerId(visionPortalIDs[0])
                    .setAutoStartStreamOnBuild(true)
                    .setAutoStopLiveView(false)
                    .setShowStatsOverlay(true)
                    .addProcessor(this.ballPipeline)
                    .addProcessor(this.rampPipeline)
                    .build();
        }

        if (hardware.backCamera != null) {
            Log.i("CameraSubsytem", "back camera built" );
            backPortal = new VisionPortal.Builder()
                    .setCamera(hardware.backCamera)
                    .setCameraResolution(new Size(1280, 800))
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .setLiveViewContainerId(visionPortalIDs[1])
                    .setAutoStartStreamOnBuild(false)
                    .setAutoStopLiveView(false)
                    .setShowStatsOverlay(true)
                    .addProcessor(tagProcessor)
                    .build();
        }
    }

    // =======================
    // processor
    // =======================
    private AprilTagProcessor createAprilTagProcessor() {
        AprilTagProcessor processor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .setLensIntrinsics(910.121, 910.121, 648.374, 394.354)
                .build();

        processor.setDecimation(1f);
        processor.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_IPPE_SQUARE);
        return processor;
    }

    private BallDetectionPipeline createBallPipeline() {
        BallDetectionPipeline pipeline = new BallDetectionPipeline(
                ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY,
                -1, -1, -1,
                Color.rgb(255, 120, 31),
                Color.rgb(255, 255, 255),
                Color.rgb(3, 227, 252)
        );

        pipeline.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                MIN_CONTOUR_AREA,
                MAX_CONTOUR_AREA
        ));

        pipeline.setSort(new ColorBlobLocatorProcessor.BlobSort(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                SortOrder.DESCENDING
        ));

        return pipeline;
    }

    private RampPipeline createRampPipeline()
    {
        return new RampPipeline();
    }

    // =======================
    // periodic
    // =======================
    @Override
    public void periodic() {
        if(frontPortal != null) {
            frontPortal.setProcessorEnabled(ballPipeline, CVMode.equals(FRONT_CV_MODE.FAR));
            frontPortal.setProcessorEnabled(rampPipeline, CVMode.equals(FRONT_CV_MODE.RAMP));
        }

        if (backPortal == null || tagProcessor == null) return;

        detections = tagProcessor.getDetections();
        AprilTagDetection localizationTag = null;

        for (AprilTagDetection tag : detections) {
            if (isGlyphTag(tag)) {
                handleGlyph(tag);
                continue;
            }

            localizationTag = tag;

        }

        handleRelocalization(localizationTag);

        //add some check here on which pieplines running or smth
    }

    // =======================
    // helper
    // =======================
    private boolean isGlyphTag(AprilTagDetection tag) {
        return tag.id >= 21 && tag.id <= 23;
    }

    public void setGlyph(GLYPH glyph) {
        gameGlyph = glyph;
        baseGlyph = glyph;
    }

    public void setBaseGlyph(GLYPH glyph)
    {
        baseGlyph = glyph;
    }

    public GLYPH getBaseGlyph()
    {
        return baseGlyph;
    }

    public void setGameGlyph(GLYPH glyph) {
        gameGlyph = glyph;
    }

    private void handleGlyph(AprilTagDetection tag) {
        if (!scanForGlyphs) return;

        String name = VisionConstants.APRILTAG.tagMap.get(tag.id);

        gameGlyph = GLYPH.valueOf(name);
        robot.telemetry.addData("Glyph", gameGlyph);
    }

    public Pose2d offsetByBallCoords(Pose2d pose){
        BallDetectionPipeline.BlobImpl blob = this.ballPipeline.getChosenBlob();
        if (blob == null) {
            return pose;
        }
        Pose2d tempPos = pose.copy();
//        double pixelX = blob.getCircle().getCenter().x;
        double pixelX = blob.getCenter().x;
        double offset = ballPipeline.pixelXtoRealX(pixelX);
        tempPos.y += offset;
        Log.d(TAG, "Ball Pixel Offset (in): " + offset);
        Log.d(TAG, "New Pose: " + tempPos);
        return tempPos;

    }

    public boolean hasBlob()
    {
        return ballPipeline.getChosenBlob() != null;
    }

    public void resetBlob() {
        ballPipeline.unlockChosenBlob();
    }


    private void handleRelocalization(AprilTagDetection tag) {
        if (!robot.getState().equals(RobotState.SCANNING) || tag == null || !relocalizationEnabled) {
            relocalizeSucceeded = false;
            relocalizeTimer.reset();
            return;
        }

        setAprilTagsEnabled(true);

        if (relocalizeTimer.milliseconds() > RELOCALIZE_WINDOW_MS) {
            robot.robotState = RobotState.RESTING;
            return;
        }

        if (tag != null && tag.robotPose != null &&
                robot.follower.getVelocity().getMagnitude() < VELOCITY_THRESHOLD) {

            relocalize(tag);
            relocalizeSucceeded = true;
        }
    }

    private void relocalize(AprilTagDetection tag) {
        double heading = robot.follower.poseTracker.getPose().getHeading();

        Pose raw = new Pose(
                72 + tag.robotPose.getPosition().y,
                72 - tag.robotPose.getPosition().x,
                heading
        );

        Pose offset = new Pose(CAMERA_OFFSET_IN, 0, 0)
                .rotate(raw.getHeading(), false);

        Pose finalPose = raw.minus(offset).setHeading(heading);

        debugLastDetection = finalPose;
        debugDetectionTime = System.currentTimeMillis();

        robot.follower.poseTracker.setCurrentPoseWithOffset(finalPose);
    }

    private void applyCameraExposureSettings() {
        try {
            ExposureControl exposure = frontPortal.getCameraControl(ExposureControl.class);
            GainControl gain = frontPortal.getCameraControl(GainControl.class);

            exposure.setMode(ExposureControl.Mode.Manual);
            exposure.setExposure(EXPOSURE_US, TimeUnit.MICROSECONDS);
            gain.setGain(GAIN);
        } catch (Exception ignored) {}
    }
    public void setBallPipelineEnabled(boolean state){
        if(state) {
            setCVMode(FRONT_CV_MODE.FAR);
            return;
        }
        if(CVMode.equals(FRONT_CV_MODE.FAR))
        {
            CVMode = FRONT_CV_MODE.NONE;
        }
    }

    public void close() {
        if (frontPortal != null) {
            frontPortal.close();
            frontPortal = null;
        }
        if (backPortal != null) {
            backPortal.close();
            backPortal = null;
        }

        if (tagProcessor != null) {
            // Allow the GC to reclaim atag resources
            tagProcessor = null;
            detections = null;
        }
    }


    // =======================
    // getter setter
    // =======================

    public BallColor[] getGlyphCharArray() {
        return gameGlyph == null ? null : gameGlyph.colors;
    }

    public FRONT_CV_MODE getCVMode()
    {
        return CVMode;
    }

    //if you are looking for where setBallPipeline went its now set CV mode
    //set it to be either far, ramp, or none

    public void setCVMode(FRONT_CV_MODE mode)
    {
        CVMode = mode;
    }

    public void setAprilTagsEnabled(boolean enabled) {
        if (backPortal != null) {
            backPortal.setProcessorEnabled(tagProcessor, enabled);
        }
    }

    public GLYPH getGlyph() {
        return gameGlyph;
    }

    public boolean didRelocalize() {
        return relocalizeSucceeded;
    }

    public int getBallsSeen(){
        return ballsSeen;
    }

    public void setBallsSeen(int ballsOnRamp){
        this.ballsSeen = ballsOnRamp;
    }

    public void setGlyphScanningEnabled(boolean state)
    {
        this.scanForGlyphs = state;
    }
}