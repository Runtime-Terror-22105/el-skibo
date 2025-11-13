package org.firstinspires.ftc.teamcode.robot.init;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.drive.localizer.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.robot.drive.mecanum.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.robot.hardware.sensors.TerrorPinpoint;
import org.firstinspires.ftc.teamcode.robot.subsystems.HangSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.vision.CameraSubsystem;
import org.opencv.core.Point;


/**
 * A class containing all the robot's subsystems.
 */
@Config
public class Robot {
    // States
    public RobotState robotState = null /*RobotState.RESTING*/;

    // Subsystems
    public MecanumDrivetrain drivetrain = null;
    public ShooterSubsystem shooter;
    public SpindexerSubsystem spindexer;


    public CameraSubsystem camera;
    public HangSubsystem hang;
    public IntakeSubsystem intake;

    // Localizer
    public PinpointLocalizer pinpoint;
    public LocalizationSubsystem localizer;
    public static double PINPOINT_X_OFFSET = 102.5;
    public static double PINPOINT_Y_OFFSET = -170;


    // Camera stuff TODO
//    public TerrorCameraVisionPortal camera;

    // Other misc public objects
    public FtcDashboard dashboard;
    public MultipleTelemetry telemetry;
    public RobotHardware hardware;

    public void init(@NonNull RobotHardware hardware, @NonNull Telemetry tele) {
        // Save local copy of RobotHardware class
        this.hardware = hardware;

        // Set up dashboard stuff
        this.dashboard = FtcDashboard.getInstance();
        this.telemetry = new MultipleTelemetry(tele, dashboard.getTelemetry());

        // Initialize the localizer
//        if (hardware.pinpoint != null) {
//            this.pinpoint = new PinpointLocalizer(hardware.pinpoint/*, hardware.imu*/);
//            pinpoint.init(new PinpointLocalizer.Parameters(
//                    PINPOINT_X_OFFSET, PINPOINT_Y_OFFSET,
//                    TerrorPinpoint.GoBildaOdometryPods.goBILDA_4_BAR_POD,
//                    TerrorPinpoint.EncoderDirection.FORWARD, TerrorPinpoint.EncoderDirection.REVERSED
//            ));
//        }
//        localizer = new LocalizationSubsystem(new Pose2d(new Point(0.0,0.0), 0.0), this.hardware, this);

        // Initialize the drivetrain
        this.drivetrain = new MecanumDrivetrain(
                hardware.motorRearLeft,
                hardware.motorFrontLeft,
                hardware.motorRearRight,
                hardware.motorFrontRight
        );
//        this.shooter = new ShooterSubsystem(hardware);
        this.spindexer = new SpindexerSubsystem(hardware);
//        this.intake = new IntakeSubsystem(hardware);
//        this.hang = new HangSubsystem(hardware);

        // Set up the camera
        if (hardware.fieldCamera != null) {
//            this.camera = new TerrorCameraVisionPortal.Builder()
//                    .setCamera(hardware.fieldCamera)
//                    .setCameraResolution(new Size(320, 240))
//                    .enableLiveView(true)
////                    .detectAprilTags()
//                    .detectColorBlobs(matchColors)
//                    .flip()
//                    .build();
        }
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
}
