package org.firstinspires.ftc.teamcode.robot.init;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.HangSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.LightControl;
import org.firstinspires.ftc.teamcode.robot.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.shooter.ShooterLookupTable;
import org.firstinspires.ftc.teamcode.robot.subsystems.vision.CameraSubsystem;
import org.firstinspires.ftc.teamcode.util.FastTelemetryImpl;


/**
 * A class containing all the robot's subsystems.
 */
@Config
public class Robot extends com.seattlesolvers.solverslib.command.Robot {
    // only use for debug cus aadit says static vars are sus
    public static MultipleTelemetry debugTelemetry;

    // No-op telemetry disables all telemetry output.
    public static boolean USE_NOOP_TELEMETRY = false;
    public static boolean USE_FAST_TELEMETRY = true;

    // States
    public RobotState robotState = RobotState.RESTING;

    //Team
    public Team color;
    // Subsystems
    public Follower follower;
    public DriveSubsystem drive;
    public ShooterSubsystem shooter;
    public SpindexerSubsystem spindexer;

    public static boolean shootInTapeZone = false;
    private final PolygonZone closeLaunchZone = new PolygonZone(new Point(144, 144), new Point(72, 72), new Point(0, 144));

    public static double ROBOT_WIDTH = 16.05118;
    public static double ROBOT_LENGTH = 17.15272;
    public static double ROBOT_TOLERANCE_X = 5.0; // a buffer for x direction
    public static double ROBOT_TOLERANCE_Y = 5.0;
    public final PolygonZone robotZone = new PolygonZone(ROBOT_WIDTH + ROBOT_TOLERANCE_X, ROBOT_LENGTH + ROBOT_TOLERANCE_Y);


    public CameraSubsystem camera;
    public HangSubsystem hang;
    public IntakeSubsystem intake;
    public LightControl lightControl;

    public Pose2d goalPos;
    private boolean autoSort;


    // Camera stuff TODO
//    public TerrorCameraVisionPortal camera;

    // Other misc public objects
    public FtcDashboard dashboard;
    public MultipleTelemetry telemetry;
    public RobotHardware hardware;
    public LinearOpMode opMode;

    private FastTelemetryImpl fastTelemetryImpl = null;

    public void init(@NonNull RobotHardware hardware, @NonNull LinearOpMode opMode) {
        CommandScheduler.getInstance().reset();

        this.opMode = opMode;

        // Save local copy of RobotHardware class
        this.hardware = hardware;

        // Set up dashboard stuff
        this.dashboard = FtcDashboard.getInstance();
        if (USE_NOOP_TELEMETRY) {
            fastTelemetryImpl = null;
            telemetry = new MultipleTelemetry();
        } else {
            Telemetry driverHubTelemetry;
            if (USE_FAST_TELEMETRY) {
                fastTelemetryImpl = new FastTelemetryImpl(opMode);
                driverHubTelemetry = fastTelemetryImpl;
            } else {
                fastTelemetryImpl = null;
                driverHubTelemetry = opMode.telemetry;
            }
            this.telemetry = new MultipleTelemetry(driverHubTelemetry, dashboard.getTelemetry());
        }
        debugTelemetry = telemetry;

        // Initialize the drivetrain
        this.follower = Constants.createFollower(hardware.hwMap);
        this.follower.breakFollowing();

        // NB: SubsystemBase will automatically register the subsystems for us
        this.drive = new DriveSubsystem(this);
        this.shooter = new ShooterSubsystem(hardware, this);
        this.spindexer = new SpindexerSubsystem(hardware, this);
        this.intake = new IntakeSubsystem(this);
        this.hang = new HangSubsystem(this);
        this.lightControl = new LightControl(hardware,this);

        // Other
        this.setAutoSort(false);

//        // Set up the camera
//        if (hardware.frontCamera != null || hardware.backCamera != null) {
//
//            this.camera = new CameraSubsystem(this,hardware, CameraSubsystem.LiveViewSettings.FIELD);
////            this.camera = new TerrorCameraVisionPortal.Builder()
////                    .setCamera(hardware.fieldCamera)
////                    .setCameraResolution(new Size(320, 240))
////                    .enableLiveView(true)
//////                    .detectAprilTags()
////                    .detectColorBlobs(matchColors)
////                    .flip()
////                    .build();
//        } else {
//            this.camera = new CameraSubsystem();
//        }
        this.camera = new CameraSubsystem(this,hardware,CameraSubsystem.LiveViewSettings.FIELD);
    }

    /**
     * Commits hardware writes (e.g. setting motor powers).
     */
    public void write() {
        hardware.write();
    }

    /**
     * Returns the robot's current state.
     * @return the robot's current state.
     */
    public RobotState getState() {
        return robotState;
    }

    public void setAutoSort(boolean autoSort, boolean inferMissingColor) {
        this.autoSort = autoSort;
        this.spindexer.inferMissingColorToSort = inferMissingColor;

        if (autoSort) {
            this.shooter.shooterLookupTable = ShooterLookupTable.SORTED_TABLE;
        }
    }

    public void setAutoSort(boolean autoSort) {
        this.setAutoSort(autoSort, true);
    }

    public void toggleAutoSort() {
        this.setAutoSort(!this.autoSort, this.spindexer.inferMissingColorToSort);
    }

    public boolean getAutoSort() {
        return this.autoSort;
    }

    public boolean getShootInTapeZone()
    {
        return shootInTapeZone;
    }

    public void setShootInTapeZone(boolean setting)
    {
        shootInTapeZone = setting;
    }

    public boolean isInTapeZone() {
//        return closeLaunchZone.contains(robotZone.getPosition());
        //note that there is another method called isFullyInside, make of that what you will
        return robotZone.isInside(closeLaunchZone);
    }

    public void close() {
        // Allow stuff to be reclaimed
        telemetry = null;
        follower = null;
        drive = null;
        shooter = null;
        spindexer = null;
        intake = null;
        hang = null;
        lightControl = null;

        if (fastTelemetryImpl != null) {
            fastTelemetryImpl.close();
            fastTelemetryImpl = null;
        }

        if (camera != null) {
            camera.close();
            camera = null;
        }

        if (hardware != null) {
            hardware.close();
            hardware = null;
        }

        // we call these and hope the jvm listens to us
        //
        // We do this because some objects (camera stuff) define `finalize` methods.
        System.gc();
        System.runFinalization();
    }
}
