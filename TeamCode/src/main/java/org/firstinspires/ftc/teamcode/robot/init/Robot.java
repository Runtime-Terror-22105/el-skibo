package org.firstinspires.ftc.teamcode.robot.init;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.drive.localizer.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.robot.drive.mecanum.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.robot.hardware.sensors.TerrorPinpoint;
import org.firstinspires.ftc.teamcode.robot.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.TurretSubsystem;


/**
 * A class containing all the robot's subsystems.
 */
@Config
public class Robot {
    private static boolean isInitialized = false;

    // States
    public static RobotState robotState = null /*RobotState.RESTING*/;

    // Subsystems
    public static MecanumDrivetrain drivetrain;
    public static TurretSubsystem turret;
    public static ShooterSubsystem shooter;
    public static SpindexerSubsystem spindexer;

    // Localizer
    public static PinpointLocalizer localizer;
    public static double PINPOINT_X_OFFSET = 102.5;
    public static double PINPOINT_Y_OFFSET = -170;

    // Camera stuff TODO
//    public TerrorCameraVisionPortal camera;

    // Other misc public objects
    public static FtcDashboard dashboard;
    public static MultipleTelemetry telemetry;

    public static void assertInitialized() {
        if (!isInitialized) {
            throw new IllegalStateException("Robot not initialized");
        }
    }

    public static void init(@NonNull Telemetry tele) {
        RobotHardware.assertInitialized();
        isInitialized = true;

        // Set up dashboard stuff
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(tele, dashboard.getTelemetry());

        // Initialize the localizer
        if (RobotHardware.pinpoint != null) {
            localizer = new PinpointLocalizer(RobotHardware.pinpoint/*, hardware.imu*/);
            localizer.init(new PinpointLocalizer.Parameters(
                    PINPOINT_X_OFFSET, PINPOINT_Y_OFFSET,
                    TerrorPinpoint.GoBildaOdometryPods.goBILDA_4_BAR_POD,
                    TerrorPinpoint.EncoderDirection.FORWARD, TerrorPinpoint.EncoderDirection.REVERSED
            ));
        }

        // Initialize the drivetrain
        drivetrain = new MecanumDrivetrain(
                RobotHardware.motorRearLeft,
                RobotHardware.motorFrontLeft,
                RobotHardware.motorRearRight,
                RobotHardware.motorFrontRight
        );

        turret = new TurretSubsystem();
        shooter = new ShooterSubsystem();
        spindexer = new SpindexerSubsystem();

        // Set up the camera
        if (RobotHardware.cameraName != null) {
//            this.camera = new TerrorCameraVisionPortal.Builder()
//                    .setCamera(hardware.cameraName)
//                    .setCameraResolution(new Size(320, 240))
//                    .enableLiveView(true)
////                    .detectAprilTags()
//                    .detectColorBlobs(matchColors)
//                    .flip()
//                    .build();
        }
    }

    /**
     * Updates all subsystems.
     */
    public void update() {
        // TODO: update all subsystems here
        RobotHardware.write();
    }
}
