package org.firstinspires.ftc.teamcode.robot.init;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.math.NumCompare;
import org.firstinspires.ftc.teamcode.robot.hardware.TerrorLight;
import org.firstinspires.ftc.teamcode.robot.hardware.TerrorPublisher;
import org.firstinspires.ftc.teamcode.robot.hardware.motors.TerrorMotorNormal;
import org.firstinspires.ftc.teamcode.robot.hardware.motors.TerrorServo;
import org.firstinspires.ftc.teamcode.robot.hardware.sensors.TerrorAnalogEncoder;
import org.firstinspires.ftc.teamcode.robot.hardware.sensors.TerrorEncoder;
import org.firstinspires.ftc.teamcode.util.Profiler;

import java.util.Arrays;
import java.util.List;

/**
 * A class containing all the robot's hardware.
 */
@Config
public class RobotHardware {
    public static String FRONT_WEBCAM_NAME = "Front Webcam";
    public static String BACK_WEBCAM_NAME = "Back Webcam";

    // Drivetrain motors & servos
    // NB: Use pedro pathing's Follower rather than access these directly.
    public TerrorMotorNormal motorFrontLeft;
    public TerrorMotorNormal motorRearRight;
    public TerrorMotorNormal motorFrontRight;
    public TerrorMotorNormal motorRearLeft;

    // Turret
    public TerrorServo turretYawLeft;  // rotates the turret yaw
    public TerrorServo turretYawRight; // rotates the turret yaw


    // gobuilda pwm lights
    public TerrorLight lights;

    // Shooter
    public TerrorMotorNormal shooterLeft;  // powers the flywheel
    public TerrorMotorNormal shooterRight; // powers the flywheel
    public TerrorServo shooterPitch;// the hood for the shooter, changes its pitch
    public TerrorEncoder shooterEncoder;   // i forgot to write the comment

    // Spindexer
    public static double SPINDEXER_ENCODER_OFFSET_DEGREES = -19;
    public static boolean SPINDEXER_ENCODER_REVERSED = true;
    public TerrorMotorNormal spindexer;
    public TerrorServo wallServoLeft;
    public TerrorServo wallServoRight;
    public TerrorServo intakeServoLeft;
    public TerrorServo intakeServoRight;
    public TerrorServo transferRampServo;
    public TerrorAnalogEncoder spindexerEncoder;
    public ColorSensorManager colorSensors;

    // Intake
    public TerrorMotorNormal intake;

    public TerrorServo pto;

    // Camera
    public int cameraMonitorViewId;
    public WebcamName frontCamera;
    public WebcamName backCamera;
//    private TerrorCameraVisionPortal camera;

    // Sensors
    public IMU imu;

    // Lynx stuff
    public List<LynxModule> allHubs;
    public LynxModule controlHub;

    // Other
    public HardwareMap hwMap;
    private final TerrorPublisher publisher = new TerrorPublisher();

    // Voltage monitoring
    public double nominalVoltage = 12.5;
    public double initialVoltage;
    private double currentVoltage = Double.NaN;
    private long lastVoltageTime = -1;

    public enum HardwareOptions {
        CAMERA
    }

    public void init(@NonNull HardwareMap hwMap, @NonNull LynxModule.BulkCachingMode bulkCachingMode, HardwareOptions... options) {
        this.hwMap = hwMap;

        // Initialize the drivetrain motors
        motorFrontLeft = new TerrorMotorNormal(
                hwMap, "motorFrontLeft",
                0.05,
                1.0
        );
        motorFrontRight = new TerrorMotorNormal(
                hwMap, "motorFrontRight",
                0.05,
                1.0
        );
        motorRearRight = new TerrorMotorNormal(
                hwMap, "motorRearRight",
                0.05,
                1.0
        );
        motorRearLeft = new TerrorMotorNormal(
                hwMap, "motorRearLeft",
                0.05,
                1.0
        );
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setDirection(FORWARD);
        motorFrontRight.setDirection(REVERSE);
        motorRearLeft.setDirection(FORWARD);
        motorRearRight.setDirection(REVERSE);
        this.publisher.subscribe(1, motorFrontLeft, motorFrontRight, motorRearLeft, motorRearRight);

        // Initialize the turret
        this.turretYawLeft = new TerrorServo(hwMap, "turretLeft", 0.0025);
        this.turretYawRight = new TerrorServo(hwMap, "turretRight", 0.0025);
        this.turretYawLeft.setPwmRange(500, 2500);
        this.turretYawRight.setPwmRange(500, 2500);
        this.publisher.subscribe(5, turretYawLeft, turretYawRight);


        // Initialize the shooter
        this.shooterLeft = new TerrorMotorNormal(
                hwMap, "shooterLeft",
                0.0003,
                1.0
        );
        this.shooterRight = new TerrorMotorNormal(
                hwMap, "shooterRight",
                0.0003,
                1.0
        );
        this.shooterEncoder = new TerrorEncoder(motorFrontLeft);
        this.shooterEncoder.setDirection(TerrorEncoder.Direction.REVERSE);// TODO: figure out which motor has the encoder
        this.publisher.subscribe(5, shooterLeft, shooterRight);

        this.shooterLeft.setDirection(FORWARD);
        this.shooterRight.setDirection(REVERSE);
//        this.shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        this.shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.shooterPitch = new TerrorServo(hwMap, "hood", 0.003);
        this.publisher.subscribe(5, shooterPitch);


        // Initialize the spindexer
        this.spindexer = new TerrorMotorNormal(
                hwMap, "spindexer",
                0.01,
                1.0
        );
        this.spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.publisher.subscribe(10, spindexer);

        this.lights = new TerrorLight(hwMap.get(Servo.class, "lights"));
        this.publisher.subscribe(11, lights);

        this.colorSensors = new ColorSensorManager(hwMap,
                "leftSensor",
                "topSensor",
                "rightSensor"
        );

        this.wallServoLeft = new TerrorServo(hwMap, "wallLeft");
        this.wallServoRight = new TerrorServo(hwMap, "wallRight");
        this.intakeServoLeft = new TerrorServo(hwMap, "intakeLeft");
        this.intakeServoRight = new TerrorServo(hwMap, "intakeRight");
        this.transferRampServo = new TerrorServo(hwMap, "ramp");
        this.publisher.subscribe(10, wallServoLeft, wallServoRight, transferRampServo);
        this.publisher.subscribe(10, intakeServoLeft, intakeServoRight);

        // gear ratio for spindexer:motor is 5.6:1, motor itself is geared 5.2:1 (which is 1+46/11),
        // and motor has 28 ticks per revolution
        // https://www.gobilda.com/5202-series-yellow-jacket-planetary-gear-motor-5-2-1-ratio-1150-rpm-3-3-5v-encoder/
        this.spindexerEncoder = new TerrorAnalogEncoder(hwMap.get(AnalogInput.class, "spindexEncoder"), SPINDEXER_ENCODER_REVERSED);
        this.spindexerEncoder.setOffset(Math.toRadians(SPINDEXER_ENCODER_OFFSET_DEGREES));

//        this.spindexerEncoder.stop_and_reset();
        // TODO: figure out spindexer encoder direction

        // Initialize the intake
        this.intake = new TerrorMotorNormal(
                hwMap, "intake",
                0.05,
                1.0
        );
        this.intake.setDirection(REVERSE);
        this.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.publisher.subscribe(10, intake);

        this.pto = new TerrorServo(hwMap, "pto");
        this.pto.setDirection(Servo.Direction.FORWARD);
        this.publisher.subscribe(10, this.pto);

        // Other things
        if (Arrays.stream(options).anyMatch(opt -> opt == HardwareOptions.CAMERA)) {
            this.initCamera();
        }
        this.initLynx(bulkCachingMode);
        this.initImu();

        this.initialVoltage = getCurrentVoltage();
    }

    public double getCurrentVoltage() {
        // Only read voltage max a few times per second
        long time = System.currentTimeMillis();
        if (time - lastVoltageTime > 300 || Double.isNaN(currentVoltage)) {
            currentVoltage = controlHub.getInputVoltage(VoltageUnit.VOLTS);
            lastVoltageTime = time;
        }
        return currentVoltage;
    }

    public double getVoltageScale() {
        return nominalVoltage / NumCompare.clamp(getCurrentVoltage(), 9.5, 14.0);
    }

    public void write() {
        Profiler.push("publisher");
        this.publisher.write();
        Profiler.pop();
        this.colorSensors.update();
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
        this.frontCamera = hwMap.get(WebcamName.class, FRONT_WEBCAM_NAME);
        this.backCamera = hwMap.get(WebcamName.class, BACK_WEBCAM_NAME);

//        this.camera = new TerrorCameraVisionPortal.Builder()
//                .setCamera(fieldCamera)
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

    private void initImu() {
        this.imu = hwMap.get(IMU.class, "imu");
        this.imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));
    }

    public void close() {

    }
}
