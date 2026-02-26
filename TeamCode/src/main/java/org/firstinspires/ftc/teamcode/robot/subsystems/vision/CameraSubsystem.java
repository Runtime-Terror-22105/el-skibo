package org.firstinspires.ftc.teamcode.robot.subsystems.vision;

import android.graphics.Color;
import android.util.Log;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.math.Coordinate;
import org.firstinspires.ftc.teamcode.pedroPathing.FtcDashDrawing;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.Profiler;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

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



    //roi
    double top = 1;
    double right = 1;

    double left = -1;
    double bottom = -1;

    int frontCameraWidth = 320;

    private final ColorBlobLocatorProcessor purpleBlobProcessor = new ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
            .setRoi(ImageRegion.asUnityCenterCoordinates(left,top,right,bottom)) //i lowk dunno this does
            .setDrawContours(true)
            .setBoxFitColor(0)
            .setCircleFitColor(Color.rgb(255,255,255))
            .setBlurSize(5)
            .setDilateSize(15)
            .setErodeSize(15)
            .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
            .build();

    private final ColorBlobLocatorProcessor greenBlobProcessor = new ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(ColorRange.ARTIFACT_GREEN)
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
            .setRoi(ImageRegion.asUnityCenterCoordinates(left,top,right,bottom)) //i lowk dunno this does
            .setDrawContours(true)
            .setBoxFitColor(0)
            .setCircleFitColor(Color.rgb(255,255,255))
            .setBlurSize(5)
            .setDilateSize(15)
            .setErodeSize(15)
            .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
            .build();



//    private VisionPipeline pipeline = new VisionPipeline(webcam);

    public CameraSubsystem() {
        this.vPortalFront = null;
        this.vPortalBack = null;
        this.shouldScanForGlyphs = true;
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
//        this.aTagProcessor = new AprilTagProcessorDash(createAprilTagProcessor());


        VisionPortal.Builder vPortalFrontBuilder = new VisionPortal.Builder()
                .setCamera(hardware.frontCamera)
                .setCameraResolution(new Size(frontCameraWidth, 240))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setLiveViewContainerId(visionPortalIDs[0])
                .addProcessors(purpleBlobProcessor,greenBlobProcessor);

        VisionPortal.Builder vPortalBackBuilder = new VisionPortal.Builder()
                .setCamera(hardware.backCamera)
                .setCameraResolution(new Size(1280, 800))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setLiveViewContainerId(visionPortalIDs[1])
                .addProcessors(this.backTagProcessor);

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
            this.vPortalFront = vPortalFrontBuilder.build();
            if (!USE_LIVE_VIEW) vPortalFront.stopLiveView();
        }
        if (hardware.backCamera != null) {
            this.vPortalBack = vPortalBackBuilder.build();
            if (!USE_LIVE_VIEW) vPortalBack.stopLiveView();
        }

//        FtcDashboard.getInstance().startCameraStream(vPortalField, 0);
        this.shouldScanForGlyphs = true;

        setAprilTagsEnabled(true);
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
        processor.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_IPPE_SQUARE);
        return processor;
    }

    public void setAprilTagsEnabled(boolean enabled) {
        if (vPortalFront != null) {
//            vPortalFront.setProcessorEnabled(frontTagProcessor, enabled);
        }
        if (vPortalBack != null) {
            vPortalBack.setProcessorEnabled(backTagProcessor, enabled);
        }
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
            robot.telemetry.addData("greenblob",greenBlobProcessor.getBlobs());
            robot.telemetry.addData("purpleblob",purpleBlobProcessor.getBlobs());

            if(!usingBackCamera || backTagProcessor.getDetections().isEmpty())
            {
//                this.detections = frontTagProcessor.getDetections();
            }
            if(!usingFrontCamera) //eventually add some check for || no balls cv detected
            {

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

            if(localizationTag != null)
            {
            Log.d("YAEHEYAEYEAH.", (localizationTag != null) + String.valueOf(localizationTag.robotPose != null) + (robot.follower.getVelocity().getMagnitude() < VELOCITY_THRESHOLD));
                robot.telemetry.addData("isRobotPoseReal", localizationTag.robotPose != null);
                robot.telemetry.addData("robotVelocityMag", robot.follower.getVelocity().getMagnitude() < VELOCITY_THRESHOLD);
            }



            Log.d("CameraSubsystem", "localizationTag != null: " + (localizationTag != null));
            Log.d("CameraSubsystem", "localizationTag.robotPose != null: " + (localizationTag != null && localizationTag.robotPose != null));
            Log.d("CameraSubsystem", "robot.follower.getVelocity().getMagnitude() < VELOCITY_THRESHOLD: "
                    + (robot.follower.getVelocity().getMagnitude() < VELOCITY_THRESHOLD));
            Log.d("CameraSubsystem", "Relocalization conditions met: " + ((localizationTag != null && localizationTag.robotPose != null)
                    && (robot.follower.getVelocity().getMagnitude() < VELOCITY_THRESHOLD)));
            if (localizationTag != null && localizationTag.robotPose != null
                    && (robot.follower.getVelocity().getMagnitude() < VELOCITY_THRESHOLD)) {
                Log.d("CameraSubsystem", "Relocalizing with tag " + localizationTag.id);
                relocalize(localizationTag);
            }

            if (debugLastDetection != null) {
                // Fade color from red to black as detection gets older
                int ageMs = (int) (System.currentTimeMillis() - debugDetectionTime);
                int red = Math.max(0, 255 - ageMs / 5);
                FtcDashDrawing.drawRobot(debugLastDetection != null ? debugLastDetection : new Pose(0, 0, 0), String.format("#%02X0000", red));
            }
        }
    }

    public boolean hasBlobs()
    {
        List<ColorBlobLocatorProcessor.Blob> blobs = purpleBlobProcessor.getBlobs();
        blobs.addAll(greenBlobProcessor.getBlobs());
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                300,500000,blobs);
        return !blobs.isEmpty();
    }

    /**
     <p>don't use unless you've confirmed there are blobs via hasBlobs()</p>
     */
    private Coordinate getLargestBlobCoordinate()
    {
        List<ColorBlobLocatorProcessor.Blob> blobs = purpleBlobProcessor.getBlobs();
        blobs.addAll(greenBlobProcessor.getBlobs());

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                300,500000,blobs);

        for(ColorBlobLocatorProcessor.Blob blob: blobs)
        {
            Circle circle = blob.getCircle();
            double circularity = blob.getCircularity();
            float radius = circle.getRadius();
            double x = circle.getX();
            double y = circle.getY();
            double contourArea = blob.getContourArea();
            double circleArea = Math.pow(radius,2)*Math.PI;
        }

        //i dont know if this is true or not but i think blob(0) is the biggest one
        ColorBlobLocatorProcessor.Blob targetBlob = blobs.get(0);

        //me when i lie
        return new Coordinate(5000,5000);

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