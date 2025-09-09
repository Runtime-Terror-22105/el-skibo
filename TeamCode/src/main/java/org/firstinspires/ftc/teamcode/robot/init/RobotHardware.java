package org.firstinspires.ftc.teamcode.robot.init;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.hardware.TerrorPublisher;
import org.firstinspires.ftc.teamcode.robot.hardware.motors.TerrorMotorNormal;
import org.firstinspires.ftc.teamcode.robot.hardware.motors.TerrorServo;
import org.firstinspires.ftc.teamcode.robot.hardware.sensors.TerrorPinpoint;

import java.util.Arrays;
import java.util.List;

/**
 * A class containing all the robot's hardware.
 */
@Config
public class RobotHardware {
    // Drivetrain motors & servos
    public static TerrorMotorNormal motorFrontLeft;
    public static TerrorMotorNormal motorRearRight;
    public static TerrorMotorNormal motorFrontRight;
    public static TerrorMotorNormal motorRearLeft;

    // Turret
    public static TerrorServo turretYawLeft;
    public static TerrorServo turretYawRight;
    public static TerrorServo turretPitch;

    // Shooter
    public static TerrorMotorNormal shooterLeft;
    public static TerrorMotorNormal shooterRight;

    // Spindexer
    public static TerrorMotorNormal spindexerRotate;

    // Camera
    public static int cameraMonitorViewId;
    public static WebcamName cameraName;
//    private TerrorCameraVisionPortal camera;

    // Sensors
    public static TerrorPinpoint pinpoint;

    // Lynx stuff
    public static List<LynxModule> allHubs;
    public static LynxModule controlHub;

    // Other
    public static HardwareMap hwMap;
    private static final TerrorPublisher publisher = new TerrorPublisher();

    public enum HardwareOptions {
        CAMERA,
        PINPOINT
    }

    public static void assertInitialized() {
        if (hwMap == null) {
            throw new IllegalStateException("RobotHardware not initialized");
        }
    }

    public static void init(@NonNull HardwareMap hwMap, @NonNull LynxModule.BulkCachingMode bulkCachingMode, HardwareOptions... options) {
        RobotHardware.hwMap = hwMap;

        // Initialize the drivetrain motors
        motorFrontLeft = new TerrorMotorNormal(
                (DcMotorEx) hwMap.get(DcMotor.class, "motorFrontLeft"),
                0.05,
                1.0
        );
        motorFrontRight = new TerrorMotorNormal(
                (DcMotorEx) hwMap.get(DcMotor.class, "motorFrontRight"),
                0.05,
                1.0
        );
        motorRearRight = new TerrorMotorNormal(
                (DcMotorEx) hwMap.get(DcMotor.class, "motorRearRight"),
                0.05,
                1.0
        );
        motorRearLeft = new TerrorMotorNormal(
                (DcMotorEx) hwMap.get(DcMotor.class, "motorRearLeft"),
                0.05,
                1.0
        );
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // TODO: figure out drive motor directions
//        motorFrontLeft.setDirection(REVERSE);
//        motorRearLeft.setDirection(REVERSE);
//        motorFrontRight.setDirection(FORWARD);
//        motorRearRight.setDirection(FORWARD);

        publisher.subscribe(4, motorFrontLeft, motorFrontRight, motorRearLeft, motorRearRight);

//        imu = hwMap.get(IMU.class, "imu");

        // Initialize the turret
        turretYawLeft = new TerrorServo(hwMap.get(Servo.class, "turretYawLeft"));
        turretYawRight = new TerrorServo(hwMap.get(Servo.class, "turretYawRight"));
        turretPitch = new TerrorServo(hwMap.get(Servo.class, "turretPitch"));
        publisher.subscribe(5, turretYawLeft, turretYawRight, turretPitch);

        // Initialize the shooter
        shooterLeft = new TerrorMotorNormal(
                (DcMotorEx) hwMap.get(DcMotor.class, "shooterLeft"),
                0.05,
                1.0
        );
        shooterRight = new TerrorMotorNormal(
                (DcMotorEx) hwMap.get(DcMotor.class, "shooterRight"),
                0.05,
                1.0
        );

        // TODO: figure out shooter motor directions
//        shooterLeft.setDirection(REVERSE);
//        shooterRight.setDirection(FORWARD);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        publisher.subscribe(5, shooterLeft, shooterRight);

        // Initialize the spindexer
        spindexerRotate = new TerrorMotorNormal(
                (DcMotorEx) hwMap.get(DcMotor.class, "spindexerRotate"),
                0.05,
                1.0
        );
        spindexerRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexerRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        publisher.subscribe(10, spindexerRotate);

        // Other things
        if (Arrays.stream(options).anyMatch(opt -> opt == HardwareOptions.CAMERA)) {
            initCamera();
        }
        initLynx(bulkCachingMode);

        // Other Sensors
        if (Arrays.stream(options).anyMatch(opt -> opt == HardwareOptions.PINPOINT)) {
            pinpoint = hwMap.get(TerrorPinpoint.class, "pinpoint");
        }
    }

    public static void write() {
        publisher.write();
    }

    private static void initLynx(LynxModule.BulkCachingMode bulkCachingMode) {
        // Initialize Lynx stuff
        allHubs = hwMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            if (hub.isParent() && LynxConstants.isEmbeddedSerialNumber(hub.getSerialNumber())) {
                controlHub = hub;
            }
            hub.setBulkCachingMode(bulkCachingMode);
        }
    }

    private static void initCamera() {
        cameraMonitorViewId = hwMap
                .appContext
                .getResources()
                .getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hwMap.appContext.getPackageName()
                );
        cameraName = hwMap.get(WebcamName.class, "Webcam 1");

//        camera = new TerrorCameraVisionPortal.Builder()
//                .setCamera(cameraName)
//                .setCameraResolution(new Size(1280, 800))
//                .detectAprilTags()
//                .init();
//
//        if (camera.tagProcessor == null) {
//            throw new IllegalStateException("AprilTag processor not initialized!");
//        }
////        camera.tagProcessor.setDecimation(???); // TODO: tune decimation value
//        camera.tagProcessor.setPoseSolver(AprilTagProcessor.PoseSolver.APRILTAG_BUILTIN);
    }
}
