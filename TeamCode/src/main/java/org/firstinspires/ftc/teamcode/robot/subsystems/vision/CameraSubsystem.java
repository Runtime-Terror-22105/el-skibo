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
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
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

    // NOTE: This will need to be adjusted depending on the lighting conditions of the field. The value is set to a default that should work in most conditions, but may need to be tweaked.
    public static int WHITE_BALANCE_TEMPERATURE = 3000; // front camera: 2800-6500K
    public static long EXPOSURE_US = 166666; // front camera: 200 - 1000000us (1s)
    public static int GAIN = 50; // front camera: 0-128

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

    private boolean ballCountChanged = false;

    public GLYPH gameGlyph;
    public FRONT_CV_MODE CVMode = FRONT_CV_MODE.NONE;

    private int ballsSeen = 0;
    private int lastBallsSeen = ballsSeen;
    private final ElapsedTime relocalizeTimer = new ElapsedTime();

    // =======================
    // debug stuff
    // =======================
    private Pose debugLastDetection = null;
    private long debugDetectionTime = 0;

    // =======================
    // const
    // =======================
    public static double MIN_CONTOUR_AREA = 200;
    public static double MAX_CONTOUR_AREA = 100000;

    public static int RELOCALIZE_WINDOW_MS = 200;
    public static double CAMERA_OFFSET_IN_X = -8;
    public static double CAMERA_OFFSET_IN_Y = 1.86;

    public static double VELOCITY_THRESHOLD = 5.0;

    private boolean isManualWhiteBalanceSet;
    private boolean isManualExposureSet;
    private boolean isManualGainSet;

    public enum GLYPH {
        PPG(BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN),
        PGP(BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE),
        GPP(BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE);

        public final BallColor[] colors;
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
                    .setAutoStopLiveView(true)
                    .setShowStatsOverlay(true)
                    .addProcessor(this.ballPipeline)
                    .addProcessor(this.rampPipeline)
                    .build();
//            setCameraSettings();
        }

        if (hardware.backCamera != null) {
            Log.i("CameraSubsytem", "back camera built" );
            backPortal = new VisionPortal.Builder()
                    .setCamera(hardware.backCamera)
                    // Logs say: Supported resolutions for MJPEG are: [1600x1200 @ 25FPS], [3264x2448 @ 15FPS], [2592x1944 @ 15FPS], [2048x1536 @ 15FPS], [1920x1080 @ 25FPS], [1280x960 @ 25FPS], [1280x720 @ 25FPS], [1024x768 @ 25FPS], [800x600 @ 25FPS], [640x480 @ 25FPS], [320x240 @ 25FPS],
                    // Logs say: Supported resolutions for YUY2 are: [1920x1080 @ 5FPS], [3264x2448 @ 1FPS], [2592x1944 @ 2FPS], [2048x1536 @ 3FPS], [1600x1200 @ 5FPS], [1280x960 @ 5FPS], [1280x720 @ 10FPS], [1024x768 @ 10FPS], [800x600 @ 25FPS], [640x480 @ 25FPS], [320x240 @ 25FPS],
                    // 640x480 is the best one that isn't super high resolution while still being very good, but 320x240 is most ideal for memory
                    // Btw MJPEG leads to like 10-15ms higher looptimes, likely because the compression takes a while
                    .setCameraResolution(new Size(320, 240))
                    .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                    .setLiveViewContainerId(visionPortalIDs[1])
                    .setAutoStartStreamOnBuild(true)
                    .setAutoStopLiveView(false)
                    .setShowStatsOverlay(true)
                    .addProcessor(tagProcessor)
                    .build();
        }
    }

    /**
     * <p>Note: The stream being enabled/disabled is distinct from whether or not it shows
     * up in the Live View on the driver station.</p>
     * <p></p>
     * <p>Disabling live view merely ensures it won't be shown to us, but the stream will still be
     * running and can be processed by our CV pipelines. This is useful for saving CPU resources
     * if we only want to use the camera for processing and not for streaming to the driver station.</p>
     * @param enabled Whether or not the front camera stream should be enabled
     */
    public void setFrontCameraStreamEnabled(boolean enabled) {
        if (frontPortal == null) return;

        if (enabled) {
            frontPortal.resumeStreaming();
        } else {
            frontPortal.stopStreaming();
        }
    }

    /**
     * <p>Note: The stream being enabled/disabled is distinct from whether or not it shows
     * up in the Live View on the driver station.</p>
     * <p></p>
     * <p>Disabling live view merely ensures it won't be shown to us, but the stream will still be
     * running and can be processed by our CV pipelines. This is useful for saving CPU resources
     * if we only want to use the camera for processing and not for streaming to the driver station.</p>
     * @param enabled Whether or not the back camera stream should be enabled
     */
    public void setBackCameraStreamEnabled(boolean enabled) {
        if (backPortal == null) return;

        if (enabled) {
            backPortal.resumeStreaming();
        } else {
            backPortal.stopStreaming();
        }
    }

    public void setCameraSettings() {
        try {
            if (frontPortal == null) return;

            if (!isManualWhiteBalanceSet) {
                this.isManualWhiteBalanceSet = setManualWhiteBalance();
                if (this.isManualWhiteBalanceSet) Log.i(TAG, "Manual white balance set successfully");
            }

            if (!isManualExposureSet) {
                this.isManualExposureSet = setManualExposure();;
                if (this.isManualExposureSet) Log.i(TAG, "Manual exposure settings applied");
            }

            if (!isManualGainSet) {
                this.isManualGainSet = setManualGain();
                if (this.isManualGainSet) Log.i(TAG, "Manual gain settings applied");
            }
        } catch (Exception e) {
            Log.e(TAG, "Error setting camera settings: " + e.getMessage());
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
        robot.telemetry.addData("Balls Seen", getBallsSeen());

        if(frontPortal != null) {
            frontPortal.setProcessorEnabled(ballPipeline, CVMode.equals(FRONT_CV_MODE.FAR));
            frontPortal.setProcessorEnabled(rampPipeline, CVMode.equals(FRONT_CV_MODE.RAMP));
            if(frontPortal.getProcessorEnabled(rampPipeline))
            {
                this.ballsSeen = rampPipeline.getBalls();
                setBallCountChanged(lastBallsSeen != this.ballsSeen);
                this.lastBallsSeen = this.ballsSeen;
            }

        }



        // ensures camera settings are set in case they weren't already
//        setCameraSettings();

        try {
            // todo: move this stuff to setcamerasettings and only call it when we need to change settings, not every frame
            frontPortal.getCameraControl(ExposureControl.class).setMode(ExposureControl.Mode.Auto);
            frontPortal.getCameraControl(WhiteBalanceControl.class).setMode(WhiteBalanceControl.Mode.AUTO);
        } catch (Exception e) {
            //Log.e(TAG, "Error checking camera control modes: " + e.getMessage());
        }

        if (backPortal == null || tagProcessor == null || !backPortal.getProcessorEnabled(tagProcessor)) return;

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

    public void setBallCountChanged(boolean state)
    {
        this.ballCountChanged = state;
    }

    public boolean getBallCountChanged()
    {
        return this.ballCountChanged;
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

        Pose offset = new Pose(CAMERA_OFFSET_IN_X, CAMERA_OFFSET_IN_Y, 0)
                .rotate(raw.getHeading(), false);

        Pose finalPose = raw.minus(offset).setHeading(heading);

        debugLastDetection = finalPose;
        debugDetectionTime = System.currentTimeMillis();

        robot.follower.poseTracker.setCurrentPoseWithOffset(finalPose);
    }

    private boolean setManualWhiteBalance() {
        WhiteBalanceControl whiteBalanceControl = frontPortal.getCameraControl(WhiteBalanceControl.class);
        if (whiteBalanceControl == null) {
            Log.w(TAG, "Camera does not support white balance control");
            return false;
        }
        boolean success = whiteBalanceControl.setMode(WhiteBalanceControl.Mode.MANUAL);
        if (!success) {
            Log.w(TAG, "Failed to set white balance mode to manual");
            return false;
        }

        Log.d(TAG, "Minimum and maximum white balance temperatures: " + whiteBalanceControl.getMinWhiteBalanceTemperature() + "K - " + whiteBalanceControl.getMaxWhiteBalanceTemperature() + "K");

        success = whiteBalanceControl.setWhiteBalanceTemperature(WHITE_BALANCE_TEMPERATURE);
        if (!success) {
            Log.w(TAG, "Failed to set white balance temperature");
            return false;
        }
        return true;
    }

    private boolean setManualExposure() {
        ExposureControl exposure = frontPortal.getCameraControl(ExposureControl.class);

        boolean success = exposure.setMode(ExposureControl.Mode.Manual);
        if (!success) {
            Log.w(TAG, "Camera does not support manual exposure control");
            return false;
        }

        Log.d(TAG, "Minimum and maximum exposure (us): " + exposure.getMinExposure(TimeUnit.MICROSECONDS) + " - " + exposure.getMaxExposure(TimeUnit.MICROSECONDS));

        success = exposure.setExposure(EXPOSURE_US, TimeUnit.MICROSECONDS);
        if (!success) {
            Log.w(TAG, "Failed to set manual exposure");
            return false;
        }
        return true;
    }

    private boolean setManualGain() {
        GainControl gain = frontPortal.getCameraControl(GainControl.class);
        if (gain == null) {
            Log.w(TAG, "Camera does not support gain control");
            return false;
        }

        Log.d(TAG, "Minimum and maximum gain: " + gain.getMinGain() + " - " + gain.getMaxGain());

        boolean success = gain.setGain(GAIN);
        if (!success) {
            Log.w(TAG, "Failed to set manual gain");
            return false;
        }
        return true;
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