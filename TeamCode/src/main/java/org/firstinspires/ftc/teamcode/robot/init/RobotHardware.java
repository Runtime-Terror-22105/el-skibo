package org.firstinspires.ftc.teamcode.robot.init;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxVoltageSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
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
import org.firstinspires.ftc.teamcode.robot.hardware.motors.TerrorSparkMiniMotor;
import org.firstinspires.ftc.teamcode.robot.hardware.sensors.TerrorAnalogEncoder;
import org.firstinspires.ftc.teamcode.robot.hardware.sensors.TerrorEncoder;
import org.firstinspires.ftc.teamcode.robot.hardware.sensors.TerrorPinpoint;
import org.firstinspires.ftc.teamcode.robot.hardware.sensors.TerrorProximitySensor;

import java.util.Arrays;
import java.util.List;

/**
 * A class containing all the robot's hardware.
 */
@Config
public class RobotHardware {
    public static double ARM_PITCH_ENCODER_OFFSET = -5.6486-1.9792-0.075;
    public static double PITCH_VOLTAGE_BASELINE = 11.5;

    // Drivetrain motors & servos
    public TerrorMotorNormal motorFrontLeft = null;
    public TerrorMotorNormal motorRearRight = null;
    public TerrorMotorNormal motorFrontRight = null;
    public TerrorMotorNormal motorRearLeft = null;

//    public IMU imu;

    // Pink arm stuff
    public TerrorSparkMiniMotor armPitchMotorLeft = null;
    public TerrorSparkMiniMotor armPitchMotorRight = null;
    public TerrorAnalogEncoder armPitchEncoder = null;
    public TerrorSparkMiniMotor armExtensionMotorLeft = null;
    public TerrorSparkMiniMotor armExtensionMotorRight = null;
    public TerrorEncoder armExtensionEncoder = null;

    // InOutTake
    public TerrorServo pitchServo;

    public TerrorServo turretServo;
    public TerrorCRServo intakeServo;

    // Drivetrain PTO for hang
    public TerrorServo dtPtoLeft;
    public TerrorServo dtPtoRight;

    // L2 Hang Servos
    public TerrorCRServo l2HangServoLeft;
    public TerrorCRServo l2HangServoRight;

    // Other servos
    public TerrorServo sweeper;

    // Camera
    public int cameraMonitorViewId;
    public WebcamName cameraName;
//    private TerrorCameraVisionPortal camera;

    // Sensors
//    public PhotonLynxVoltageSensor voltageSensor;
//    public TerrorSparkFunOTOS otos;
    public TerrorPinpoint pinpoint;
    public TerrorProximitySensor proximitySensor;

    // Lynx stuff
    public List<LynxModule> allHubs = null;
    public LynxModule controlHub = null;
    public LynxVoltageSensor voltageSensor;

    // Other
    public HardwareMap hwMap;
    private TerrorPublisher publisher = new TerrorPublisher();

    private double initialVoltage;

    public static enum HardwareOptions {
        CAMERA,
        PINPOINT
    }

    public void init(@NonNull HardwareMap hwMap, @NonNull LynxModule.BulkCachingMode bulkCachingMode, HardwareOptions... options) {
        this.hwMap = hwMap;

        this.voltageSensor = (LynxVoltageSensor) hwMap.voltageSensor.iterator().next();
        this.initialVoltage = voltageSensor.getVoltage();
        double pitchPowerScale = PITCH_VOLTAGE_BASELINE/initialVoltage;
//        double powerScale = Math.min(VOLTAGE_BASELINE/initialVoltage, 1.0);

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

        this.motorFrontLeft.setDirection(REVERSE);
        this.motorRearLeft.setDirection(REVERSE);
        this.motorFrontRight.setDirection(FORWARD);
        this.motorRearRight.setDirection(FORWARD);

        this.publisher.subscribe(4, motorFrontLeft, motorFrontRight, motorRearLeft, motorRearRight);


//        this.imu = hwMap.get(IMU.class, "imu");


        // Initialize the pink arm motors and sensors
        this.armPitchMotorLeft = new TerrorSparkMiniMotor(
                hwMap.get(CRServo.class, "armPitchMotorLeft"),
                0.02,
                pitchPowerScale
        );
        this.armPitchMotorRight = new TerrorSparkMiniMotor(
                hwMap.get(CRServo.class, "armPitchMotorRight"),
                0.02,
                pitchPowerScale
        );
        this.armPitchEncoder = new TerrorAnalogEncoder(hwMap.get(AnalogInput.class, "armPitchEncoder"), true);
//         this.armPitchEncoder.setOffset(2*Math.PI - Math.toRadians(36));
        this.armPitchEncoder.setOffset(ARM_PITCH_ENCODER_OFFSET);
        this.armExtensionMotorLeft = new TerrorSparkMiniMotor(
                hwMap.get(CRServo.class, "armExtensionMotorLeft"),
                0.02,
                1.0 //powerScale
        );
        this.armExtensionMotorRight = new TerrorSparkMiniMotor(
                hwMap.get(CRServo.class, "armExtensionMotorRight"),
                0.02,
                1.0 //powerScale
        );

        this.armPitchMotorLeft.setDirection(REVERSE);
        this.armExtensionMotorLeft.setDirection(FORWARD);
        this.armExtensionMotorRight.setDirection(REVERSE);

//        this.armExtensionEncoder = new TerrorEncoder(motorFrontRight); // might need to change to motor 2
        this.armExtensionEncoder = new TerrorEncoder(motorRearRight); // might need to change to motor 2
        this.armExtensionEncoder.setDirection(TerrorEncoder.Direction.REVERSE);

        this.publisher.subscribe(5, armPitchMotorLeft, armPitchMotorRight);
        this.publisher.subscribe(3, armExtensionMotorLeft, armExtensionMotorRight);

        // Initialize the inouttake servos and sensors
        this.pitchServo = new TerrorServo(hwMap.get(Servo.class, "pitchServo"));
        this.turretServo = new TerrorServo(hwMap.get(Servo.class, "turretServo"));
        this.intakeServo = new TerrorCRServo(hwMap.get(CRServo.class, "intakeServo"),0.05, 1.0);

        this.intakeServo.setDirection(REVERSE);
        this.publisher.subscribe(9, pitchServo, intakeServo, turretServo);

        this.l2HangServoLeft = new TerrorCRServo(
                hwMap.get(CRServo.class, "l2HangServoLeft"),
                0.02,
                1.0
        );
        this.l2HangServoRight = new TerrorCRServo(
                hwMap.get(CRServo.class, "l2HangServoRight"),
                0.02,
                1.0
        );
        this.publisher.subscribe(10, l2HangServoLeft, l2HangServoRight);

        // PTO servos
        this.dtPtoLeft = new TerrorServo(hwMap.get(Servo.class, "dtPtoLeft"));
        this.dtPtoRight = new TerrorServo(hwMap.get(Servo.class, "dtPtoRight"));
        this.publisher.subscribe(7, dtPtoLeft);
        this.publisher.subscribe(8, dtPtoRight);


        // Other servos
        this.sweeper = new TerrorServo(hwMap.get(Servo.class, "sweeper"));
        this.publisher.subscribe(20, sweeper);

//         // Other things
        if (Arrays.stream(options).anyMatch(opt -> opt == HardwareOptions.CAMERA)) {
            this.initCamera();
        }
        this.initLynx(bulkCachingMode);

        // Other Sensors
        if (Arrays.stream(options).anyMatch(opt -> opt == HardwareOptions.PINPOINT)) {
            this.pinpoint = hwMap.get(TerrorPinpoint.class, "pinpoint");
        }
        this.proximitySensor = new TerrorProximitySensor(hwMap, "proximitySensor", 0.20);
//        this.otos = this.hwMap.get(TerrorSparkFunOTOS.class, "sensor_otos");
//        this.voltageSensor = hwMap.getAll(PhotonLynxVoltageSensor.class).iterator().next();
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
