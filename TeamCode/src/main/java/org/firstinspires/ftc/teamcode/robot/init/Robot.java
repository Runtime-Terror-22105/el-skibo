package org.firstinspires.ftc.teamcode.robot.init;

import static org.firstinspires.ftc.teamcode.robot.subsystems.PinkArm.HIGH_BUCKET_PITCH_DEGREES;
import static org.firstinspires.ftc.teamcode.robot.subsystems.PinkArm.HORIZONTAL_LIMIT;
import static org.firstinspires.ftc.teamcode.robot.subsystems.PinkArm.MAX_PITCH_DEGREES;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.drive.localizer.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.robot.drive.mecanum.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.robot.match.MatchColors;
import org.firstinspires.ftc.teamcode.robot.match.SampleColor;
import org.firstinspires.ftc.teamcode.robot.hardware.sensors.TerrorPinpoint;
import org.firstinspires.ftc.teamcode.robot.subsystems.PTO;
import org.firstinspires.ftc.teamcode.robot.subsystems.PinkArm;
import org.firstinspires.ftc.teamcode.robot.subsystems.Skibbidintake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Sweeper;
import org.firstinspires.ftc.teamcode.robot.vision.TerrorCameraVisionPortal;
/**
 * A class containing all the robot's subsystems.
 */
@Config
public class Robot {
    // States
    public RobotState robotState = RobotState.RESTING;

    // Subsystems
    public MecanumDrivetrain drivetrain = null;
    public PinkArm pinkArm = null;
    public Skibbidintake skibbidintake = null;

    // Minor subsystems
    public PTO dtPto = null;
    public Sweeper sweeper;

    public SampleColor sample_color;

    // Localizer
    public PinpointLocalizer localizer;
    public static double PINPOINT_X_OFFSET = 102.5;
    public static double PINPOINT_Y_OFFSET = -170;

    // Camera stuff
    public TerrorCameraVisionPortal camera;

    // Other misc public objects
    public FtcDashboard dashboard;
    public MultipleTelemetry telemetry;
    public RobotHardware hardware;

    public void init(@NonNull RobotHardware hardware, @NonNull Telemetry tele, SampleColor sampleColor) {
        this.init(hardware, tele, new MatchColors(
                sampleColor,
                sampleColor == SampleColor.RED ? SampleColor.BLUE : SampleColor.RED
        ));
    }

    public void init(@NonNull RobotHardware hardware, @NonNull Telemetry tele, MatchColors matchColors) {
        this.init(hardware, tele, matchColors, false);
    }

    public void init(@NonNull RobotHardware hardware, @NonNull Telemetry tele, MatchColors matchColors, boolean isAuto) {
        // Save local copy of RobotHardware class
        this.hardware = hardware;

        // Set up dashboard stuff
        this.dashboard = FtcDashboard.getInstance();
        this.telemetry = new MultipleTelemetry(tele, dashboard.getTelemetry());

        // Initialize the localizer
        if (hardware.pinpoint != null) {
            this.localizer = new PinpointLocalizer(hardware.pinpoint/*, hardware.imu*/);
            localizer.init(new PinpointLocalizer.Parameters(
                    PINPOINT_X_OFFSET, PINPOINT_Y_OFFSET,
                    TerrorPinpoint.GoBildaOdometryPods.goBILDA_4_BAR_POD,
                    TerrorPinpoint.EncoderDirection.FORWARD, TerrorPinpoint.EncoderDirection.REVERSED
            ));
        }

        // Initialize the drivetrain
        this.drivetrain = new MecanumDrivetrain(
                hardware.motorRearLeft,
                hardware.motorFrontLeft,
                hardware.motorRearRight,
                hardware.motorFrontRight
        );

        // Set up the camera
        if (hardware.cameraName != null) {
            this.camera = new TerrorCameraVisionPortal.Builder()
                    .setCamera(hardware.cameraName)
                    .setCameraResolution(new Size(320, 240))
                    .enableLiveView(true)
//                    .detectAprilTags()
                    .detectColorBlobs(matchColors)
                    .flip()
                    .build();
        }

        // Set up subsytems
        this.pinkArm = new PinkArm(hardware);
        this.dtPto = new PTO(this);
        this.skibbidintake = new Skibbidintake(hardware);
        this.sweeper = new Sweeper(hardware, isAuto);
    }

    public RobotState getState() {
        return this.robotState;
    }

    /**
     * Change the robot's state if it is not in the correct state.
     * @param desiredState The desired state of the robot.
     */
    public void setState(RobotState desiredState) {
        this.robotState = desiredState;

        switch (this.robotState) {
            case RESTING:
//            case RESTING_RESET_ENCODER:
                goToRestingState();
                break;
            case INTAKE:
                goToIntakeState();
                break;
            case HIGH_BUCKET:
                goToHighBucketState();
                break;
            case LOW_BUCKET:
                goToLowBucketState();
                break;
            case AUTO_RESTING:
                goToAutoRestingState();
                break;
            case SAMPLE_INTERMEDIARY_UP:
                goToSampleIntermediaryUpState();
                break;
            case SAMPLE_INTERMEDIARY_DOWN:
                goToSampleIntermediaryDownState();
                break;
        }

        pinkArm.update();
    }

    private void goToAutoRestingState() {
//        pinkArm.goDown();
        pinkArm.setPitchTarget(0);
        skibbidintake.setPitchIntake();
    }

    private void goToRestingState() {
        // move pink arm (pitch and extension 0)
        pinkArm.goDown(true);
        skibbidintake.resetAngle();
        skibbidintake.setPitchResting();
        skibbidintake.stop();
    }

    private void goToIntakeState() {
        // move pink arm (arm pitched down, slightly extended ~5-10 inches)
        pinkArm.setPitchTarget(0);
        pinkArm.setExtensionTarget(HORIZONTAL_LIMIT * 0.75);
        // Middle - still over the bar but partially down (saves time)
        skibbidintake.setPitchMiddle();
    }

    private void goToHighBucketState() {
        // move pink arm (arm pitched up ~90-95 deg, max extension ~40-50 in)
        pinkArm.goHighBasket();
        skibbidintake.setPitchMiddle();
        skibbidintake.resetAngle();
//        skibbidintake.hold();
    }

    private void goToLowBucketState() {
        // move pink arm (arm pitched up ~90-95 deg, low bucket level etension)
        pinkArm.goLowBasket();
        skibbidintake.resetAngle();
        skibbidintake.setPitchScoring();
//        skibbidintake.hold();
    }

    private void goToSampleIntermediaryDownState() {
        pinkArm.setExtensionTarget(0);
        pinkArm.setPitchTarget(Math.toRadians(HIGH_BUCKET_PITCH_DEGREES-15));
        skibbidintake.resetAngle();
        skibbidintake.setPitchMiddle();
        skibbidintake.stop();
    }

    private void goToSampleIntermediaryUpState() {
        pinkArm.setPitchTarget(Math.toRadians(MAX_PITCH_DEGREES));
        pinkArm.setExtensionTarget(0);
        skibbidintake.resetAngle();
        skibbidintake.setPitchMiddle();
//        skibbidintake.hold();
    }

    /**
     * Updates all subsystems.
     */
    public void update() {
        pinkArm.update();
        skibbidintake.update(pinkArm.getExtensionPosition());
        dtPto.update();
        sweeper.update();
        hardware.write();
    }
}
