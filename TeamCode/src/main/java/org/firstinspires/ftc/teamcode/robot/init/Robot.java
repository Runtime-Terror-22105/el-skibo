package org.firstinspires.ftc.teamcode.robot.init;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.HangSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.LightControl2;
import org.firstinspires.ftc.teamcode.robot.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.vision.CameraSubsystem;


/**
 * A class containing all the robot's subsystems.
 */
@Config
public class Robot extends com.seattlesolvers.solverslib.command.Robot {
    // only use for debug cus aadit says static vars are sus
    public static MultipleTelemetry debugTelemetry;

    // States
    public RobotState robotState = RobotState.RESTING;

    //Team
    public Team color;
    // Subsystems
    public Follower follower;
    public DriveSubsystem drive;
    public ShooterSubsystem shooter;
    public SpindexerSubsystem spindexer;

    public int debugLight = 0;


    public CameraSubsystem camera;
    public HangSubsystem hang;
    public IntakeSubsystem intake;
    public LightControl2 lightControl;

    public Pose2d goalPos;
    private boolean autoSort;


    // Camera stuff TODO
//    public TerrorCameraVisionPortal camera;

    // Other misc public objects
    public FtcDashboard dashboard;
    public MultipleTelemetry telemetry;
    public RobotHardware hardware;

    public void init(@NonNull RobotHardware hardware, @NonNull Telemetry tele) {
        CommandScheduler.getInstance().reset();

        // Save local copy of RobotHardware class
        this.hardware = hardware;

        // Set up dashboard stuff
        this.dashboard = FtcDashboard.getInstance();
        this.telemetry = new MultipleTelemetry(tele, dashboard.getTelemetry());
        debugTelemetry = telemetry;

        // Initialize the drivetrain
        this.follower = Constants.createFollower(hardware.hwMap);
        this.follower.breakFollowing();

        // NB: SubsystemBase will automatically register the subsystems for us
        this.drive = new DriveSubsystem(this);
        this.shooter = new ShooterSubsystem(hardware, this);
        this.spindexer = new SpindexerSubsystem(hardware, this);
        this.intake = new IntakeSubsystem(this);
        this.hang = new HangSubsystem(hardware);
        this.lightControl = new LightControl2(hardware,this);

        // Other
        this.setAutoSort(false);

        // Set up the camera
        if (hardware.fieldCamera != null) {

            this.camera = new CameraSubsystem(this,hardware, CameraSubsystem.LiveViewSettings.FIELD);
//            this.camera = new TerrorCameraVisionPortal.Builder()
//                    .setCamera(hardware.fieldCamera)
//                    .setCameraResolution(new Size(320, 240))
//                    .enableLiveView(true)
////                    .detectAprilTags()
//                    .detectColorBlobs(matchColors)
//                    .flip()
//                    .build();
        } else {
            this.camera = new CameraSubsystem();
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

    public void setAutoSort(boolean autoSort) {
        this.autoSort = autoSort;
    }

    public void toggleAutoSort() {
        this.autoSort = !this.autoSort;
    }

    public boolean getAutoSort() {
        return this.autoSort;
    }
}
