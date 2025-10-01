package org.firstinspires.ftc.teamcode.robot.init;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.drive.localizer.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.robot.drive.mecanum.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.robot.hardware.sensors.TerrorPinpoint;
import org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;


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
    public IntakeSubsystem intake;

    // Localizer
    public PinpointLocalizer localizer;
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
        this.shooter = new ShooterSubsystem(hardware);
        this.spindexer = new SpindexerSubsystem(hardware);
        this.intake = new IntakeSubsystem(hardware);

        // Set up the camera
        if (hardware.cameraName != null) {
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
    public void setState(RobotState state){
        switch (state){
            case RESTING:
                goToRestingState();
                break;

            case INTAKING:
                goToIntakingState();
                break;
            case FULL:
                goToFullState();
                break;
            case SHOOTING:
                //same as full prob, just that spindexer could be in different pos
            // this is assuming diddy climb, could change
            case CLIMBING:
                goToClimbState();
                break;
            case DONE_CLIMB:
                goToClimbDoneState();
                break;

        }
        this.robotState = state;

    }
    public RobotState getState() {
        return this.robotState;
    }
    private void goToRestingState(){
        this.shooter.manualAim(0.0, 45.0, 0.0);
        //spindexer has funnel out
        //intake up
    }
    private void goToIntakingState(){
        //if intake is up, put down
        //spin intake in
        //spindexer has funnel out
    }
    private void goToFullState(){
        //intake spin oppisite
        //start running autoaim function in loop
    }
    private void goToClimbState(){
        //activate pto
        hardware.motorRearLeft.setPower(1.0);
        hardware.motorRearRight.setPower(1.0);
    }
    private void goToClimbDoneState(){
        hardware.motorRearLeft.setPower(0.7);
        hardware.motorRearRight.setPower(0.7);
    }

    /**
     * Commits hardware writes (e.g. setting motor powers).
     */
    public void write() {
        hardware.write();
    }
}
