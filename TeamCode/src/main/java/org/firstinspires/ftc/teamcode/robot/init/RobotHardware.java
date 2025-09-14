package org.firstinspires.ftc.teamcode.robot.init;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.hardware.TerrorPublisher;
import org.firstinspires.ftc.teamcode.robot.hardware.motors.TerrorCRServo;
import org.firstinspires.ftc.teamcode.robot.hardware.motors.TerrorMotorNormal;
import org.firstinspires.ftc.teamcode.robot.hardware.motors.TerrorServo;
import org.firstinspires.ftc.teamcode.robot.hardware.sensors.TerrorAnalogEncoder;
import org.firstinspires.ftc.teamcode.robot.hardware.sensors.TerrorEncoder;
import org.firstinspires.ftc.teamcode.robot.hardware.sensors.TerrorPinpoint;

import java.util.Arrays;
import java.util.List;

/**
 * A class containing all the robot's hardware.
 */
@Config
public class RobotHardware {
    // Drivetrain motors & servos
    public TerrorMotorNormal motorFrontLeft;
    public TerrorMotorNormal motorRearRight;
    public TerrorMotorNormal motorFrontRight;
    public TerrorMotorNormal motorRearLeft;

    // Turret
    public TerrorCRServo turretYawLeft;
    public TerrorCRServo turretYawRight;
    public TerrorServo turretPitch;
    public TerrorServo turretCam;
    public TerrorCRServo turretWheels;
    public TerrorAnalogEncoder turretYawEncoder;

    // Shooter
    public TerrorMotorNormal shooterLeft;
    public TerrorMotorNormal shooterRight;
    public TerrorEncoder shooterEncoder;

    // Spindexer
    public TerrorMotorNormal spindexerRotate;
    public TerrorAnalogEncoder spindexerEncoder;
    public ColorSensor[] spindexerColors;

    // Intake
    public TerrorMotorNormal intake;
    public TerrorServo intakeDoubleSideSwitch;

    // Camera
    public int cameraMonitorViewId;
    public WebcamName cameraName;
//    private TerrorCameraVisionPortal camera;

    // Sensors
    public TerrorPinpoint pinpoint;

    // Lynx stuff
    public List<LynxModule> allHubs;
    public LynxModule controlHub;

    // Other
    public HardwareMap hwMap;
    private final TerrorPublisher publisher = new TerrorPublisher();

    public enum HardwareOptions {
        CAMERA,
        PINPOINT
    }

    public void init(@NonNull HardwareMap hwMap, @NonNull LynxModule.BulkCachingMode bulkCachingMode, HardwareOptions... options) {
        this.hwMap = hwMap;

        // Initialize the drivetrain motors
        this.motorFrontLeft = new TerrorMotorNormal(
                (DcMotorEx) hwMap.get(DcMotor.class, "motorFrontLeft"),
                0.05,
                1.0
        );
        this.motorFrontRight = new TerrorMotorNormal(
                (DcMotorEx) hwMap.get(DcMotor.class, "motorFrontRight"),
                0.05,
                1.0
        );
        this.motorRearRight = new TerrorMotorNormal(
                (DcMotorEx) hwMap.get(DcMotor.class, "motorRearRight"),
                0.05,
                1.0
        );
        this.motorRearLeft = new TerrorMotorNormal(
                (DcMotorEx) hwMap.get(DcMotor.class, "motorRearLeft"),
                0.05,
                1.0
        );
        this.motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorRearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorRearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorRearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorRearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // TODO: figure out drive motor directions
//        this.motorFrontLeft.setDirection(REVERSE);
//        this.motorRearLeft.setDirection(REVERSE);
//        this.motorFrontRight.setDirection(FORWARD);
//        this.motorRearRight.setDirection(FORWARD);

        this.publisher.subscribe(4, motorFrontLeft, motorFrontRight, motorRearLeft, motorRearRight);

        // Initialize the turret
        this.turretYawLeft = new TerrorCRServo(
                hwMap.get(CRServo.class, "turretYawLeft"),
                0.05,
                1.0
        );
        this.turretYawRight = new TerrorCRServo(
                hwMap.get(CRServo.class, "turretYawRight"),
                0.05,
                1.0
        );
        this.turretPitch = new TerrorServo(hwMap.get(Servo.class, "turretPitch"));
        this.turretCam = new TerrorServo(hwMap.get(Servo.class, "turretCam"));
        this.turretWheels = new TerrorCRServo(
                hwMap.get(CRServo.class, "turretWheels"),
                0.05,
                1.0
        );
        this.turretYawEncoder = new TerrorAnalogEncoder(
                hwMap.get(AnalogInput.class, "turretYawEncoder"),
                false  // TODO: figure out if reversed
        );
        this.publisher.subscribe(5, turretYawLeft, turretYawRight, turretPitch, turretCam, turretWheels);

        // Initialize the shooter
        this.shooterLeft = new TerrorMotorNormal(
                (DcMotorEx) hwMap.get(DcMotor.class, "shooterLeft"),
                0.05,
                1.0
        );
        this.shooterRight = new TerrorMotorNormal(
                (DcMotorEx) hwMap.get(DcMotor.class, "shooterRight"),
                0.05,
                1.0
        );
        this.shooterEncoder = new TerrorEncoder(shooterLeft);  // TODO: figure out which motor has the encoder

        // TODO: figure out shooter motor directions
//        this.shooterLeft.setDirection(REVERSE);
//        this.shooterRight.setDirection(FORWARD);
        this.shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.publisher.subscribe(5, shooterLeft, shooterRight);

        // Initialize the spindexer
        this.spindexerRotate = new TerrorMotorNormal(
                (DcMotorEx) hwMap.get(DcMotor.class, "spindexerRotate"),
                0.05,
                1.0
        );
        this.spindexerRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.spindexerRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.spindexerEncoder = new TerrorAnalogEncoder(
                hwMap.get(AnalogInput.class, "spindexerEncoder"),
                false  // TODO: figure out if reversed
        );
        this.spindexerColors = new ColorSensor[] {
                hwMap.get(ColorSensor.class, "spindexerColor1"),
                hwMap.get(ColorSensor.class, "spindexerColor2"),
                hwMap.get(ColorSensor.class, "spindexerColor3")
        };
        for (ColorSensor sensor : spindexerColors) {
            sensor.enableLed(true);
        }
        this.publisher.subscribe(10, spindexerRotate);

        // Initialize the intake
        this.intake = new TerrorMotorNormal(
                (DcMotorEx) hwMap.get(DcMotor.class, "intake"),
                0.05,
                1.0
        );
        // TODO: figure out intake motor direction
//        this.intake.setDirection(FORWARD);
        this.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.intakeDoubleSideSwitch = new TerrorServo(hwMap.get(Servo.class, "intakeDoubleSideSwitch"));
        this.publisher.subscribe(10, intake, intakeDoubleSideSwitch);

        // Other things
        if (Arrays.stream(options).anyMatch(opt -> opt == HardwareOptions.CAMERA)) {
            this.initCamera();
        }
        this.initLynx(bulkCachingMode);

        // Other Sensors
        if (Arrays.stream(options).anyMatch(opt -> opt == HardwareOptions.PINPOINT)) {
            this.pinpoint = hwMap.get(TerrorPinpoint.class, "pinpoint");
        }
    }

    public void write() {
        this.publisher.write();
    }

    private void initLynx(LynxModule.BulkCachingMode bulkCachingMode) {
        // Initialize Lynx stuff
        this.allHubs = this.hwMap.getAll(LynxModule.class);
        for (LynxModule hub : this.allHubs) {
            if (hub.isParent() && LynxConstants.isEmbeddedSerialNumber(hub.getSerialNumber())) {
                this.controlHub = hub;
            }
            hub.setBulkCachingMode(bulkCachingMode);
        }
    }

    private void initCamera() {
        this.cameraMonitorViewId = hwMap
                .appContext
                .getResources()
                .getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hwMap.appContext.getPackageName()
                );
        this.cameraName = hwMap.get(WebcamName.class, "Webcam 1");

//        this.camera = new TerrorCameraVisionPortal.Builder()
//                .setCamera(cameraName)
//                .setCameraResolution(new Size(1280, 800))
//                .detectAprilTags()
//                .init();
//
//        if (this.camera.tagProcessor == null) {
//            throw new IllegalStateException("AprilTag processor not initialized!");
//        }
////        this.camera.tagProcessor.setDecimation(???); // TODO: tune decimation value
//        this.camera.tagProcessor.setPoseSolver(AprilTagProcessor.PoseSolver.APRILTAG_BUILTIN);
    }
}
