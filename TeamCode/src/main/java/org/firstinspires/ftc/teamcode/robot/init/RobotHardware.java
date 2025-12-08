package org.firstinspires.ftc.teamcode.robot.init;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.hardware.TerrorLight;
import org.firstinspires.ftc.teamcode.robot.hardware.TerrorPublisher;
import org.firstinspires.ftc.teamcode.robot.hardware.motors.TerrorMotorNormal;
import org.firstinspires.ftc.teamcode.robot.hardware.motors.TerrorServo;
import org.firstinspires.ftc.teamcode.robot.hardware.sensors.TerrorAnalogEncoder;
import org.firstinspires.ftc.teamcode.robot.hardware.sensors.TerrorColorSensor;
import org.firstinspires.ftc.teamcode.robot.hardware.sensors.TerrorEncoder;

import java.util.Arrays;
import java.util.List;

/**
 * A class containing all the robot's hardware.
 */
@Config
public class RobotHardware {
    // Drivetrain motors & servos
    // NB: Use pedro pathing's Follower rather than access these directly.
//    public TerrorMotorNormal motorFroTntLeft;
//    public TerrorMotorNormal motorRearRight;
//    public TerrorMotorNormal motorFrontRight;
//    public TerrorMotorNormal motorRearLeft;

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
    public static double SPINDEXER_ENCODER_OFFSET = 3.23;
    public TerrorMotorNormal spindexerRotate;
    public TerrorServo spindexerIntakeWallServo1;
    public TerrorServo spindexerIntakeWallServo2;
    public TerrorServo spindexerDiddyServo;
    public TerrorServo spindexerTransferRampServo; // todo -- in position: 0, out position: 0.3
    public TerrorAnalogEncoder spindexerEncoder;

    /*
             top (the one that shoots)
        left      right
     */
    private int colorSensorIndex;
    private TerrorColorSensor[] colorSensors;
    public TerrorColorSensor topSensor;
    public TerrorColorSensor leftSensor;
    public TerrorColorSensor rightSensor;

    // Intake
    public TerrorMotorNormal intake;
    public TerrorServo intakePitchLeft;
    public TerrorServo intakePitchRight;

    public TerrorServo spindexerPTO;

    // Camera
    public int cameraMonitorViewId;
    public WebcamName fieldCamera;
//    private TerrorCameraVisionPortal camera;

    // Sensors
    public IMU imu;

    // Lynx stuff
    public List<LynxModule> allHubs;
    public LynxModule controlHub;

    // Other
    public HardwareMap hwMap;
    private final TerrorPublisher publisher = new TerrorPublisher();

    public boolean disableColorSensor = false;

    public enum HardwareOptions {
        CAMERA
    }

    public void init(@NonNull HardwareMap hwMap, @NonNull LynxModule.BulkCachingMode bulkCachingMode, HardwareOptions... options) {
        this.hwMap = hwMap;

        // Initialize the drivetrain motors
        TerrorMotorNormal motorFrontLeft = new TerrorMotorNormal(
                (DcMotorEx) hwMap.get(DcMotor.class, "motorFrontLeft"),
                0.05,
                1.0
        );
        TerrorMotorNormal motorFrontRight = new TerrorMotorNormal(
                (DcMotorEx) hwMap.get(DcMotor.class, "motorFrontRight"),
                0.05,
                1.0
        );
        TerrorMotorNormal motorRearRight = new TerrorMotorNormal(
                (DcMotorEx) hwMap.get(DcMotor.class, "motorRearRight"),
                0.05,
                1.0
        );
        TerrorMotorNormal motorRearLeft = new TerrorMotorNormal(
                (DcMotorEx) hwMap.get(DcMotor.class, "motorRearLeft"),
                0.05,
                1.0
        );


        // Initialize the turret
        this.turretYawLeft = new TerrorServo(
                hwMap.get(Servo.class, "turretYawLeft")
        );
        this.turretYawRight = new TerrorServo(
                hwMap.get(Servo.class, "turretYawRight")
        );
        this.turretYawLeft.setPwmRange(500, 2500);
        this.turretYawRight.setPwmRange(500, 2500);
        this.publisher.subscribe(5, turretYawLeft, turretYawRight);


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
        this.shooterEncoder = new TerrorEncoder(motorRearLeft);
        this.shooterEncoder.setDirection(TerrorEncoder.Direction.REVERSE);// TODO: figure out which motor has the encoder
        this.publisher.subscribe(5, shooterLeft, shooterRight);

        // TODO: figure out shooter motor directions
        this.shooterLeft.setDirection(FORWARD);
        this.shooterRight.setDirection(FORWARD);
//        this.shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        this.shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.shooterPitch = new TerrorServo(hwMap.get(Servo.class, "shooterHood"));
        this.publisher.subscribe(5, shooterPitch);


        // Initialize the spindexer
        this.spindexerRotate = new TerrorMotorNormal(
                (DcMotorEx) hwMap.get(DcMotor.class, "spindexerRotate"),
                0.05,
                1.0
        );
        this.spindexerRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.spindexerRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.publisher.subscribe(10, spindexerRotate);

        this.lights=new TerrorLight(hwMap.get(Servo.class, "lights"));
        this.publisher.subscribe(11, lights);


        this.topSensor = new TerrorColorSensor(
                hwMap.get(RevColorSensorV3.class, "topSensor")
        );
        this.leftSensor = new TerrorColorSensor(
                hwMap.get(RevColorSensorV3.class, "leftSensor")
        );
        this.rightSensor = new TerrorColorSensor(
                hwMap.get(RevColorSensorV3.class, "rightSensor")
        );
        this.colorSensorIndex = 0;
        this.colorSensors = new TerrorColorSensor[] {
                this.leftSensor,
                this.topSensor,
                this.rightSensor
        };
        this.spindexerIntakeWallServo1 = new TerrorServo(hwMap.get(Servo.class, "spindexerIntakeWall1"));
        this.spindexerIntakeWallServo2 = new TerrorServo(hwMap.get(Servo.class, "spindexerIntakeWall2"));
        this.spindexerTransferRampServo = new TerrorServo(hwMap.get(Servo.class, "spindexerTransferRamp"));
        this.spindexerDiddyServo = new TerrorServo(hwMap.get(Servo.class, "diddyServo"));
        this.publisher.subscribe(10, spindexerIntakeWallServo1,
                spindexerIntakeWallServo2, spindexerDiddyServo, spindexerTransferRampServo);

        // gear ratio for spindexer:motor is 5.6:1, motor itself is geared 5.2:1 (which is 1+46/11),
        // and motor has 28 ticks per revolution
        // https://www.gobilda.com/5202-series-yellow-jacket-planetary-gear-motor-5-2-1-ratio-1150-rpm-3-3-5v-encoder/
        this.spindexerEncoder = new TerrorAnalogEncoder(hwMap.get(AnalogInput.class,"spindexEncoder"), true);
        this.spindexerEncoder.setOffset(SPINDEXER_ENCODER_OFFSET);
//        this.spindexerEncoder.stop_and_reset();
      // TODO: figure out spindexer encoder direction

        // Initialize the intake
        this.intake = new TerrorMotorNormal(
                (DcMotorEx) hwMap.get(DcMotor.class, "intake"),
                0.05,
                1.0
        );
        this.intake.setDirection(REVERSE);
        this.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.publisher.subscribe(10, intake);

        this.intakePitchLeft = new TerrorServo(hwMap.get(Servo.class, "intakePitchLeft"));
        this.intakePitchRight = new TerrorServo(hwMap.get(Servo.class, "intakePitchRight"));
        this.spindexerPTO = new TerrorServo(hwMap.get(Servo.class, "spindexerPTO"));
        this.publisher.subscribe(10,intakePitchRight);
        this.publisher.subscribe(10, intakePitchLeft);
        this.publisher.subscribe(10, spindexerPTO);

        // Other things
        if (Arrays.stream(options).anyMatch(opt -> opt == HardwareOptions.CAMERA)) {
            this.initCamera();
        }
        this.initLynx(bulkCachingMode);
        this.initImu();

        // Other Sensors
    }

    private void updateColorSensors() {
        // Update one color sensor per call to spread out the I2C load
        this.colorSensors[this.colorSensorIndex].update();
        this.colorSensorIndex = (this.colorSensorIndex + 1) % this.colorSensors.length;
    }

    public void write() {
        this.publisher.write();
        if (!this.disableColorSensor) {
            this.updateColorSensors();
        }
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
        this.fieldCamera = hwMap.get(WebcamName.class, "Webcam 1");

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
        this.imu = hwMap.get(IMU.class,  "imu");
        this.imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));
    }
}
