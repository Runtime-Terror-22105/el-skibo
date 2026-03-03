package org.firstinspires.ftc.teamcode.robot.subsystems.vision;

import android.graphics.Color;
import android.util.Log;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.SortOrder;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.pedroPathing.FtcDashDrawing;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.Profiler;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.ArrayList;
import java.util.Arrays;

@Config
public class CameraSubsystem extends SubsystemBase {
    /*
      front camera is primarily for ball detection
      back camera is primarily for atag relocalization
      we'll see what we can do with both
     */

    public static String TAG = "CameraSubsystem";

    public static boolean GLOBAL_DISABLE_RELOCALIZATION = false;
    public static boolean USE_LIVE_VIEW = false;

    //banks on the camera always being aligned on one of the robot's center axes
    public static double cameraOffsetInches = -8;


    public static double CONVERGENCE_RATE = 0.1;
    public static double VELOCITY_THRESHOLD = 5.0; // inches per second

    public static long EXPOSURE_MICROSECONDS = 200;
    public static int GAIN = 255;

    private AprilTagProcessorDash aTagProcessor;
//    private AprilTagProcessor frontTagProcessor;
    private AprilTagProcessor backTagProcessor;

    private boolean shouldScanForGlyphs = true;
    public boolean disableRelocalization = false;
    public boolean disableAprilTagsAfterGlyph = false;
//    private BallDetectionPipeline ballPipeline;
    public static double MIN_CONTOUR_AREA = 300;
    public static double MAX_CONTOUR_AREA = 100000;

    public enum GLYPH {
        GPP(BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE),
        PGP(BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE),
        PPG(BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN);

        public final BallColor[] colors;

        GLYPH(BallColor... colors) {
            this.colors = colors;
        }
    }

    public enum LiveViewSettings {OFF, FIELD}

    public Robot robot;
    public RobotHardware hardware;

    public GLYPH gameGlyph;

    public VisionPortal vPortalFront;
    public VisionPortal vPortalBack;

    public static boolean usingFrontCamera = true;
    public static boolean usingBackCamera = true;

    private ArrayList<AprilTagDetection> detections;

    private Pose debugLastDetection = null; // for debug only
    private long debugDetectionTime = 0; // for debug only

    private AprilTagDetection[] obeliskpair = {null,null};

    private Team team;

    private int[] visionPortalIDs;

    public Pose2d ballGoal = new Pose2d(8, 24);
    public static Pose2d ballDefaultGoal = new Pose2d(8, 24);

    private boolean doBallVision = false;

    public static double pixelValueLow = 70;
    public static double pixelValueHigh = 240;
    public static double inchesValueLow = -12;
    public static double inchesValueHigh = 12;

    private boolean exposureHasBeenSet = false;



    //roi
    double top = 1;
    double right = 1;
    double left = -1;
    double bottom = -1;

    int frontCameraWidth = 320;

    private ElapsedTime relocalizeTimer;

    public static int relocalizeTimeWindowMS = 200;

    private boolean relocalizeSucceeded = false;


//    private VisionPipeline pipeline = new VisionPipeline(webcam);

    public CameraSubsystem() {
        this.vPortalFront = null;
        this.vPortalBack = null;
        this.shouldScanForGlyphs = true;
        this.relocalizeTimer = new ElapsedTime();
    }

    public boolean getRelocalizeSucceeded(){
        return this.relocalizeSucceeded;
    }

    public CameraSubsystem(Robot robot, RobotHardware hardware, LiveViewSettings liveViewSettings) {
        this.vPortalFront = null;
        this.vPortalBack = null;
        this.visionPortalIDs =  VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);
        this.robot = robot;
        this.hardware = hardware;
        this.detections = new ArrayList<>();
//        this.frontTagProcessor = createAprilTagProcessor();
        this.backTagProcessor = createAprilTagProcessor();
//        this.ballPipeline = createBallDetectionPipeline();
//        this.aTagProcessor = new AprilTagProcessorDash(createAprilTagProcessor());

        Log.d(TAG, "Vision portal IDs: " + Arrays.toString(visionPortalIDs));
//        VisionPortal.Builder vPortalFrontBuilder = new VisionPortal.Builder()
//                .setCamera(hardware.frontCamera)
//                .setCameraResolution(new Size(frontCameraWidth, 240))
//                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
//                .setLiveViewContainerId(visionPortalIDs[0])
//                .setAutoStartStreamOnBuild(true)
//                .setAutoStopLiveView(false)
//                .setShowStatsOverlay(true)
//                .addProcessor(this.ballPipeline);

        VisionPortal.Builder vPortalBackBuilder = new VisionPortal.Builder()
                .setCamera(hardware.backCamera)
                .setCameraResolution(new Size(1280, 800))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setLiveViewContainerId(visionPortalIDs[1])
                .setAutoStartStreamOnBuild(true)
                .setAutoStopLiveView(false)
                .setShowStatsOverlay(true)
                .addProcessor(this.backTagProcessor);

        switch (liveViewSettings) {
            case FIELD:
//                vPortalFieldBuilder.enableLiveView(true);
//                        .setLiveViewContainerId(hardware.cameraMonitorViewId);
                // note: to have this appear in dashboard, you need to have the pipeline implement CameraStreamSource
                // see last year's code for reference, I'm too lazy to do this rn
//                FtcDashboard.getInstance().startCameraStream(atagPipeline, 0);
                break;
        }

        if (hardware.frontCamera != null) {
//            FtcDashboard.getInstance().startCameraStream(ballPipeline, 0);
//            this.vPortalFront = vPortalFrontBuilder.build();
//            if (!USE_LIVE_VIEW) vPortalFront.stopLiveView();
        }
        if (hardware.backCamera != null) {
            this.vPortalBack = vPortalBackBuilder.build();
            if (!USE_LIVE_VIEW) vPortalBack.stopLiveView();
        }

//        FtcDashboard.getInstance().startCameraStream(vPortalField, 0);
        this.shouldScanForGlyphs = true;

        setAprilTagsEnabled(true);
//        setBallPipelineEnabled(false);

        if (!CameraUtil.setManualExposureMode(vPortalFront)) {
            Log.e(TAG, "Failed to set manual exposure mode for front camera");
        }
        exposureHasBeenSet = false;
        this.relocalizeTimer = new ElapsedTime();
//        vPortalBack.stopStreaming();
    }

    private AprilTagProcessor createAprilTagProcessor() {
        AprilTagProcessor processor = new AprilTagProcessor.Builder()
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
        processor.setDecimation(1f);
        processor.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_IPPE_SQUARE);
        return processor;
    }

    private BallDetectionPipeline createBallDetectionPipeline() {
        BallDetectionPipeline pipeline = new BallDetectionPipeline(
                org.firstinspires.ftc.teamcode.robot.subsystems.vision.ImageRegion
                        .asImageCoordinates(0, 0, 320, 240), // the roi
                ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY, // don't do blobs from nested contours
                -1, // erodesize
                -1, // don't dilate the image
                -1, // don't blur the image
                Color.rgb(255, 120, 31), // bounding box, orange-ish color
                Color.rgb(255, 255, 255), // roi color, white
                Color.rgb(3, 227, 252) // contour color, light blue
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

    public void setBallPipelineEnabled(boolean enabled) {
//        if (vPortalFront != null) {
//            vPortalFront.setProcessorEnabled(ballPipeline, enabled);
//        }
//
//        doBallVision = enabled;
    }

    public void setAprilTagsEnabled(boolean enabled) {
        if (vPortalFront != null) {
//            vPortalFront.setProcessorEnabled(frontTagProcessor, enabled);
        }
        if (vPortalBack != null) {
            vPortalBack.setProcessorEnabled(backTagProcessor, enabled);
        }
    }

//    public void scheduleRelocalizeRequest()
//    {
////        robot.lightControl.setManualLightColor(TerrorLight.LightColors.YELLOW);
//        robot.telemetry.addLine("this is still being held");
//        this.relocalizeSucceeded = false;
//        this.relocalizeTimer.reset();
//        this.hasRelocalizeRequest = true;
//        this.robot.lightControl.setIsManualLighting(true);
//        this.robot.lightControl.setManualLightColor(TerrorLight.LightColors.RED);
//
//    }

    public void startCamera()
    {
        vPortalBack.resumeStreaming();
    }

    public void stopCamera() {
//        vPortalFront.stopStreaming();
        vPortalBack.stopStreaming();
    }

    /*

                 __
                 \/
             + yaw    -yaw
     */

    public void stopScanningForGlyphs() {
        this.shouldScanForGlyphs = false;
    }

    public void startScanningForGlyphs() {
        this.shouldScanForGlyphs = true;
    }

    public GLYPH getGlyph() {
        return gameGlyph;
    }

    public BallColor[] getGlyphCharArray() {
        return gameGlyph == null ? null : gameGlyph.colors;
    }

    public void setGlyph(GLYPH glyph) {
        gameGlyph = glyph;
//        // Log.i("CameraSubsystem", "Found glyph " + gameGlyph);
    }

    public void setTeam(Team team)
    {
        this.team = team;
    }

    public void setObeliskPairInAuto(AprilTagDetection[] pair)
    {
        Integer[] tags = {pair[0].id,pair[1].id};
        Arrays.sort(tags);
//        // Log.d(TAG,"redpair: " + VisionConstants.APRILTAG.glyphMap.get(VisionConstants.APRILTAG.RedObeliskPairs.get(Arrays.asList(tags))));
//        // Log.d(TAG,"bluepair: " + VisionConstants.APRILTAG.glyphMap.get(VisionConstants.APRILTAG.BlueObeliskPairs.get(Arrays.asList(tags))));
        switch(team)
        {
            case RED:
                gameGlyph = VisionConstants.APRILTAG.glyphMap.get(VisionConstants.APRILTAG.RedObeliskPairs.get(Arrays.asList(tags)));
//                gameGlyph = GLYPH.valueOf(VisionConstants.APRILTAG.tagMap.get(VisionConstants.APRILTAG.RedObeliskPairs.get(pair)));
                break;

            case BLUE:
                gameGlyph = VisionConstants.APRILTAG.glyphMap.get(VisionConstants.APRILTAG.BlueObeliskPairs.get(Arrays.asList(tags)));
                break;

            default:
                // Log.d("CameraSubsystem", "Can't do this, team unknown!");
                break;
        }

    }

    public Pose2d getBallCoords(){
//        BallDetectionPipeline.BlobImpl blob = this.ballPipeline.getChosenBlob();
//        if (blob == null) {
//            return ballDefaultGoal;
//        }
//        Pose2d tempPos = ballDefaultGoal.copy();
////        double offset = blob.getCenter().x;
//        double offset = ballPipeline.pixelToRealCoords(blob.getCircle().getCenter()).x;
//        tempPos.y += offset;
//        return tempPos;
        return ballDefaultGoal;
    }

    @Override
    public void periodic() {
        try (Profiler.Scope p = Profiler.enter("CameraSubsystem")) {
            if(
//                    (usingFrontCamera && vPortalFront == null) ||
                    (usingBackCamera && vPortalBack == null) ||
                    (!usingBackCamera && !usingFrontCamera))
            {
                return;
            }



//            if (!exposureHasBeenSet) {
//                if (usingFrontCamera && vPortalFront != null) {
                    if (CameraUtil.setManualExposureMode(vPortalFront)) {
                        Log.i(TAG, "Manual exposure mode set for front camera");
                    } else {
                        Log.e(TAG, "Failed to set manual exposure mode for front camera");
                    }

                    if (CameraUtil.setManualExposure(vPortalFront, (int)EXPOSURE_MICROSECONDS / 1000, GAIN)) {
                        Log.i(TAG, "Manual exposure set for front camera");
                        exposureHasBeenSet = true;
                    } else {
                        Log.e(TAG, "Failed to set manual exposure for front camera");
                    }
//                }
//            }

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
//        try {
//            ExposureControl exposure = vPortalField.getCameraControl(ExposureControl.class);
//            GainControl gain = vPortalField.getCameraControl(GainControl.class);
//            exposure.setMode(ExposureControl.Mode.Manual);
//            exposure.setExposure(EXPOSURE_MICROSECONDS, TimeUnit.MICROSECONDS);
//            gain.setGain(GAIN);
//        } catch (IllegalStateException e) {
//            // there's an error where it says that you cannot set controls until camera starts streaming
//            // todo handle ths properly and don't just do a try-catch
//            // Log.w("CameraSubsystem", e);
//        }

            robot.telemetry.addData("backTagProcessor enabled", vPortalBack.getProcessorEnabled(backTagProcessor));
            if (!vPortalBack.getProcessorEnabled(backTagProcessor)) {
                return;
            }

            this.detections = backTagProcessor.getDetections();
//            robot.telemetry.addData("blobs array",ballPipeline.getBlobs());

            if(!usingBackCamera || backTagProcessor.getDetections().isEmpty())
            {
//                this.detections = frontTagProcessor.getDetections();
            }
            if(!usingFrontCamera) //eventually add some check for || no balls cv detected
            {

            }

            if(usingFrontCamera && this.doBallVision){
                //this.ballGoal = this.getBlobCoordinates();

            }

            // Log.d(TAG, "shouldscanforglyph: " + shouldScanForGlyphs);
            //should only ever be the blue or red goal which is 20 and 24 respectively
            AprilTagDetection localizationTag = null;
            int obeliskIndex = 0;

            robot.telemetry.addData("detections",detections);

            for (AprilTagDetection tag : detections) {
                if (tag.id >= 21 && tag.id <= 23) {
                    obeliskpair[obeliskIndex] = tag;
                    obeliskIndex++;
                    if(shouldScanForGlyphs)
                    {
                        GLYPH glyphhh = GLYPH.valueOf(VisionConstants.APRILTAG.tagMap.get(tag.id));
                        this.gameGlyph = glyphhh;
                        robot.telemetry.addData("seenButUnusedGlyph", glyphhh);
                        // Log.d(TAG, "seenButUnusedGlyph: " + glyphhh);
                        if (disableAprilTagsAfterGlyph) {
                            setAprilTagsEnabled(false);
                        }
                    }
//
                } else {
                    localizationTag = tag;
                }
            }
//            if(isNearAuto) {
//                if(obeliskIndex==1)
//                {
//                        setGlyphByNormal(obeliskpair[0]);
//                }
//                else if(obeliskIndex==2)
//                {
////                    Arrays.sort(obeliskpair);
//                    // Log.d(TAG, "obeliskpair: " + Arrays.toString(obeliskpair));
//                    setObeliskPairInAuto(obeliskpair);
//                }
//            }
            // Log.d(TAG, "Glyph: " + gameGlyph);
            // Log.d(TAG, "Velocity Magnitude: " + robot.follower.getVelocity().getMagnitude());
            // Log.d(TAG, "Localization Tag: " + localizationTag);
            robot.telemetry.addData("Glyph", gameGlyph);
            robot.telemetry.addData("Velocity Magnitude", "" + robot.follower.getVelocity().getMagnitude() + " < " +
                    VELOCITY_THRESHOLD + " = " + (robot.follower.getVelocity().getMagnitude() < VELOCITY_THRESHOLD));

            if(localizationTag != null)
            {
                robot.telemetry.addData("Localization Tag id", localizationTag.id);
            }

            if(this.robot.getState().equals(RobotState.SCANNING))
            {
                setAprilTagsEnabled(true);

                if(this.relocalizeTimer.milliseconds() > relocalizeTimeWindowMS)
                {
                    this.robot.robotState = RobotState.RESTING;
//                    stopCamera();
                }

                if (localizationTag != null && localizationTag.robotPose != null
                        ){//(robot.follower.getVelocity().getMagnitude() < VELOCITY_THRESHOLD)) {
                    Log.d("CameraSubsystem", "Relocalizing with tag " + localizationTag.id);
                    relocalize(localizationTag);
                    this.relocalizeSucceeded = true;
                }

            }
            else
            {
                this.relocalizeSucceeded = false;
                this.relocalizeTimer.reset();
            }


            Log.d("CameraSubsystem", "localizationTag != null: " + (localizationTag != null));
            Log.d("CameraSubsystem", "localizationTag.robotPose != null: " + (localizationTag != null && localizationTag.robotPose != null));
            Log.d("CameraSubsystem", "robot.follower.getVelocity().getMagnitude() < VELOCITY_THRESHOLD: "
                    + (robot.follower.getVelocity().getMagnitude() < VELOCITY_THRESHOLD));
            Log.d("CameraSubsystem", "Relocalization conditions met: " + ((localizationTag != null && localizationTag.robotPose != null)
                    && (robot.follower.getVelocity().getMagnitude() < VELOCITY_THRESHOLD)));




            if(localizationTag != null)
            {
            Log.d("YAEHEYAEYEAH.", (localizationTag != null) + String.valueOf(localizationTag.robotPose != null) + (robot.follower.getVelocity().getMagnitude() < VELOCITY_THRESHOLD));
                robot.telemetry.addData("isRobotPoseReal", localizationTag.robotPose != null);
                robot.telemetry.addData("robotVelocityMag", robot.follower.getVelocity().getMagnitude() < VELOCITY_THRESHOLD);
            }





            if (debugLastDetection != null) {
                // Fade color from red to black as detection gets older
                int ageMs = (int) (System.currentTimeMillis() - debugDetectionTime);
                int red = Math.max(0, 255 - ageMs / 5);
                FtcDashDrawing.drawRobot(debugLastDetection != null ? debugLastDetection : new Pose(0, 0, 0), String.format("#%02X0000", red));
            }
        }
    }

    public boolean hasBlob()
    {
        return false;
//        return ballPipeline.getChosenBlob() != null;
    }

    public void resetBlob() {
//        ballPipeline.unlockChosenBlob();
    }

    private void relocalize(AprilTagDetection tag)
    {
//        Pose2D pose2D = new Pose2D(DistanceUnit.INCH, tag.robotPose.getPosition().x, tag.robotPose.getPosition().y, AngleUnit.RADIANS, tag.robotPose.getOrientation().getYaw(AngleUnit.RADIANS));
//        Pose rawPose = PoseConverter.pose2DToPose(pose2D, PedroCoordinates.INSTANCE);
//        Pose rawPose = new Pose(72 + tag.robotPose.getPosition().y, 72 - tag.robotPose.getPosition().x, tag.robotPose.getOrientation().getYaw(AngleUnit.RADIANS));
        double pinpointRobotHeading = robot.follower.poseTracker.getPose().getHeading();
        Pose rawPose = new Pose(72 + tag.robotPose.getPosition().y, 72 - tag.robotPose.getPosition().x, pinpointRobotHeading);
        //this funnily not a "raw pose" then but oh well noone cares

        double offsetX = cameraOffsetInches * Math.cos(rawPose.getHeading());
        double offsetY = cameraOffsetInches * Math.sin(rawPose.getHeading());

        Pose cameraOffset = new Pose(cameraOffsetInches, 0, 0).rotate(rawPose.getHeading(), false);
//        Pose cameraOffset = new Pose(offsetX,offsetY);
        Pose localizedPose = rawPose.minus(cameraOffset);
        localizedPose = localizedPose.setHeading(pinpointRobotHeading);

        robot.telemetry.addData("trying to relocalize",localizedPose);

        debugLastDetection = localizedPose;
        debugDetectionTime = System.currentTimeMillis();

        if(!GLOBAL_DISABLE_RELOCALIZATION && !disableRelocalization)
        {
            robot.follower.poseTracker.setCurrentPoseWithOffset(localizedPose);
//            robot.follower.setPose(localizedPose);
            robot.telemetry.addData("pleasework",localizedPose.toString());
            //i would be lying if i understood the diff between this and setpose
            //i was not listening to double take when i wrote i would be lying twice and now thrice in a row
        }
        robot.telemetry.addData("follower",robot.follower.getPose());

                //i'd be lying if i said i understood what this does from the old localizatoin code
//        // Apply exponential convergence
//        if (!disableRelocalization) {
//            Pose currentPose = robot.follower.poseTracker.getPose();
//            Pose convergedPose = new Pose(
//                    currentPose.getX() + CONVERGENCE_RATE * (robotPose.getX() - currentPose.getX()),
//                    currentPose.getY() + CONVERGENCE_RATE * (robotPose.getY() - currentPose.getY()),
//                    currentPose.getHeading() + CONVERGENCE_RATE * (robotPose.getHeading() - currentPose.getHeading())
//            );
//            robot.follower.poseTracker.setCurrentPoseWithOffset(convergedPose);
//        }

//        Log.d("pleasework",localizedPose.toString());
        /*
        * 73.62766055610237, 68.73611089751476, -115.51848399842707)*/
        /*pleasework: (66.54386596014463, 75.20379864761392, -124.53039951388197)*/
    }


    public void close() {
        if (vPortalFront != null) {
            vPortalFront.close();
            vPortalFront = null;
        }
        if (vPortalBack != null) {
            vPortalBack.close();
            vPortalBack = null;
        }

        if (backTagProcessor != null) {
            // Allow the GC to reclaim atag resources
            backTagProcessor = null;
            detections = null;
        }
    }

    //=======old stuff=======

    /*
     public void setGlyphByNormal(AprilTagDetection tag)
    {
        robot.telemetry.addData(TAG,"skibidi yaw:"+Math.toDegrees(tag.ftcPose.yaw));
//        robot.telemetry.addData(TAG,"skibidi pitch:"+tag.ftcPose.pitch);
        robot.telemetry.addData(TAG,"skibidi bearing:"+Math.toDegrees(tag.ftcPose.bearing));
        double tagAngle = tag.ftcPose.yaw; //TODO: replace with whichever works
        if(team.equals(Team.RED))
        {
            if(tagAngle < 0)
            {
                setGlyph(GLYPH.valueOf(VisionConstants.APRILTAG.tagMap.get(tag.id)));
            }
            else
            {
                setGlyph(VisionConstants.APRILTAG.RedIndividualPairs.get(tag.id));
            }
        }
        else if(team.equals(Team.BLUE))
        {
            if(tagAngle < 0)
            {
                setGlyph(GLYPH.valueOf(VisionConstants.APRILTAG.tagMap.get(tag.id)));
            }
            else
            {
                setGlyph(VisionConstants.APRILTAG.BlueIndividualPairs.get(tag.id));
            }
        }
    }
     */

//    private void handleLocalizationDetection(AprilTagDetection tag) {
//        Pose cameraFieldPose = new Pose(72 + tag.robotPose.getPosition().y, 72 - tag.robotPose.getPosition().x, tag.robotPose.getOrientation().getYaw(AngleUnit.RADIANS) + Math.PI);
////        FtcDashDrawing.drawDot(cameraFieldPose, "#0000FF");
//
//        double pinpointRobotHeading = robot.follower.poseTracker.getPose().getHeading();
//
//        // If the Pinpoint IMU heading is extremely off from the calculated angle, this means we've
//        // likely inited the IMU in the wrong position. So, we will do a "full localization reset"
//        // to correct it.
////        boolean isFullReset = MathFunctions.getSmallestAngleDifference(cameraRobotHeading, pinpointRobotHeading) < Math.toRadians(FULL_RESET_THRESHOLD_ANGLE);
//        double robotHeading;
//        double convergenceRate;
//        robotHeading = pinpointRobotHeading;
//        convergenceRate = CONVERGENCE_RATE;
//
//        Pose turretVector = new Pose(cameraToRobotCenterOffset.x, cameraToRobotCenterOffset.y, 0).rotate(cameraFieldPose.getHeading(), false);
//        Pose robotCenter = cameraFieldPose.minus(turretVector);
////        FtcDashDrawing.drawDot(turretCenter, "#FF00FF");
//
//        Pose robotPose = new Pose(robotCenter.getX(), robotCenter.getY(), robotHeading);
//
//        debugLastDetection = robotPose;
//        debugDetectionTime = System.currentTimeMillis();
//
//        // Apply exponential convergence
//        if (!disableRelocalization) {
//            Pose currentPose = robot.follower.poseTracker.getPose();
//            Pose convergedPose = new Pose(
//                    currentPose.getX() + convergenceRate * (robotPose.getX() - currentPose.getX()),
//                    currentPose.getY() + convergenceRate * (robotPose.getY() - currentPose.getY()),
//                    currentPose.getHeading() + convergenceRate * (robotPose.getHeading() - currentPose.getHeading())
//            );
//            robot.follower.poseTracker.setCurrentPoseWithOffset(convergedPose);
//        }
//    }

}