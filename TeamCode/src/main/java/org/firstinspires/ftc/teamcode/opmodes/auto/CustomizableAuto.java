package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.FieldConstants.AUTO_ENDING_DATA_KEY;
import static org.firstinspires.ftc.teamcode.FieldConstants.SPINDEXER_POSITION_KEY;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.skeletonarmy.marrow.prompts.BooleanPrompt;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.pedroPathing.FtcDashDrawing;
import org.firstinspires.ftc.teamcode.robot.command.intake.SetIntakePitchCommand;
import org.firstinspires.ftc.teamcode.robot.command.shooter.ShootThreeBallsCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.PrepareShootCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.WaitForSpindexerYawCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToIntakeStateCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToRestingStateCommand;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.prompts.DetailedOptionPrompt;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake.IntakePitch;

import kotlin.NotImplementedError;

@Config
@Configurable
@Autonomous(name = "Build-A-Bear Auto", group = "Auto")
public class CustomizableAuto extends LinearOpMode {
    private boolean mirror;
    private SequentialCommandGroup paths;

    public enum StartingPosition {
        NEAR,
        FAR
    }

    public enum IntakePosition {
        POSITION_1,
        POSITION_2,
        POSITION_3,
        HUMAN_PLAYER;

        @NonNull
        @Override
        public String toString() {
            switch (this) {
                case POSITION_1:
                case POSITION_2:
                case POSITION_3:
                    return "● ● ●";
                case HUMAN_PLAYER:
                    return "(human player)";
                default:
                    return super.toString();
            }
        }
    }

    public enum ShootingPosition {
        NEAR_TRIANGLE,
        FAR_TRIANGLE
    }

    public enum ParkSettings {
        NEAR_PARK,
        FAR_PARK,
        NO_PARK;

        @Override
        public String toString() {
            switch (this) {
                case NEAR_PARK:
                    return "Park on the near side";
                case FAR_PARK:
                    return "Park on the far side";
                case NO_PARK:
                    return "Don't park";
                default:
                    return super.toString();
            }
        }
    }

    public static double MAX_POWER = 1.0;

    public static Pose2d SHOOT_PRELOAD_POSE = new Pose2d(50.0, 104.644, Math.toRadians(315));

    public static Pose2d PREPARE_INTAKE_1_POSE = new Pose2d(52.598, 85.149, Math.toRadians(180));
    public static Pose2d INTAKE_1_POSE = new Pose2d(25, 85.149, Math.toRadians(180));
    public static Pose2d PUSH_GATE_POSE = new Pose2d(23, 72.827, Math.toRadians(180));
    public static Pose2d SHOOT_POSE = new Pose2d(50, 104.644, Math.toRadians(315));

    public static Pose2d PREPARE_INTAKE_2_POSE = new Pose2d(PREPARE_INTAKE_1_POSE.x, 60, Math.toRadians(180));
    public static Pose2d INTAKE_2_POSE = new Pose2d(20, 60, Math.toRadians(180));

    public static Pose2d PREPARE_INTAKE_3_POSE = new Pose2d(PREPARE_INTAKE_1_POSE.x, 37, Math.toRadians(180));
    public static Pose2d INTAKE_3_POSE = new Pose2d(20, 37, Math.toRadians(180));

    public static Pose2d NEAR_PARK_POSE = new Pose2d(52.282, 120.575, Math.toRadians(315));

    public static int PRE_INTAKE_DELAY = 0;
    public static int INTAKE_DELAY = 400;
    public static int PRELOAD_PRE_SHOOT_DELAY = 250;
    public static int PRE_SHOOT_DELAY = 0;
    public static int SHOOT_DELAY = 0;

    public static double MAX_DRIVETRAIN_POWER_INTAKING = 0.8;

    private final RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();
    private final Prompter prompter = new Prompter(this);
    private Team team;

    private long lastLoop = System.nanoTime();

    public void setTeam(Team team) {
        this.team = team;
        if (team == Team.BLUE){
            robot.goalPos = FieldConstants.BLUE_GOAL_POS;
        }
        else {
            robot.goalPos = FieldConstants.RED_GOAL_POS;
        }
        robot.goalPos = team.getGoalPos();
        this.mirror = Team.RED.equals(team);
    }

    public void setStartingPosition(StartingPosition startPos) {
        Pose2d startPose;
        if (startPos == StartingPosition.NEAR) {
            startPose = team.getStartPosNear();
        } else {
            startPose = team.getStartPosFar();
        }
        robot.follower.setStartingPose(startPose.toPedro());
    }

    public void onPromptsComplete() {
        setTeam(prompter.get("alliance"));
        setStartingPosition(prompter.get("start"));
        buildPaths();
    }

    public void buildPaths() {
        paths = new SequentialCommandGroup();
        IntakePosition[] intakePositions = {
                prompter.get("intake1"),
                prompter.get("intake2"),
                prompter.get("intake3")
        };
        ShootingPosition[] shootingPositions = {
                prompter.get("shootingPosition1"),
                prompter.get("shootingPosition2"),
                prompter.get("shootingPosition3")
        };

        // this should in theory inititially be the starting pose
        Pose lastPose = robot.follower.getPose();

        if (prompter.get("shootPreload")) {
            PathChain shootPreloadPath = robot.follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    lastPose,
                                    getShootPreloadPose()
                            )
                    )
                    .setLinearHeadingInterpolation(
                            lastPose.getHeading(),
                            getShootPreloadPose().getHeading()
                    )
                    .build();
            lastPose = getShootPreloadPose();

            SequentialCommandGroup shootPreloadCommand = new SequentialCommandGroup(
                    new ParallelCommandGroup(
                            new PrepareShootCommand(robot),
                            new FollowPathCommand(robot.follower, shootPreloadPath, true)
                    ),
                    new WaitCommand(PRELOAD_PRE_SHOOT_DELAY),
                    new ShootThreeBallsCommand(robot),
                    new WaitForSpindexerYawCommand(robot.spindexer).withTimeout(500),
                    new WaitCommand(SHOOT_DELAY)
            );

            paths.addCommands(shootPreloadCommand);
        }

        for (int i = 0; i < intakePositions.length; i++) {
            IntakePosition intakePos = intakePositions[i];
            ShootingPosition shootingPos = shootingPositions[i];

            paths.addCommands(
                    new ParallelCommandGroup(
                            new FollowPathCommand(robot.follower, robot.follower
                                    .pathBuilder()
                                    .addPath(
                                            new BezierLine(
                                                    lastPose,
                                                    getPrepareIntakePose(intakePos)
                                            )
                                    )
                                    .setLinearHeadingInterpolation(
                                            lastPose.getHeading(),
                                            getPrepareIntakePose(intakePos).getHeading()
                                    )
                                    .build()),
                            new SequentialCommandGroup(
                                    new WaitCommand(PRE_INTAKE_DELAY),
                                    new SetIntakePitchCommand(robot.intake, IntakePitch.DOWN)
                            )
                    ),
                    new ParallelCommandGroup(
                            new FollowPathCommand(robot.follower, robot.follower
                                    .pathBuilder()
                                    .addPath(
                                            new BezierLine(
                                                    getPrepareIntakePose(intakePos),
                                                    getIntakePose(intakePos)
                                            )
                                    )
                                    .setLinearHeadingInterpolation(
                                            getPrepareIntakePose(intakePos).getHeading(),
                                            getIntakePose(intakePos).getHeading()
                                    )
                                    .build()),
                            new SequentialCommandGroup(
                                    new WaitCommand(INTAKE_DELAY),
                                    new GoToIntakeStateCommand(robot)
                            )
                    ),
                    new FollowPathCommand(robot.follower, robot.follower
                            .pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            getIntakePose(intakePos),
                                            getShootingPose(shootingPos)
                                    )
                            )
                            .setLinearHeadingInterpolation(
                                    getIntakePose(intakePos).getHeading(),
                                    getShootingPose(shootingPos).getHeading()
                            )
                            .build()),
                    new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                    new WaitCommand(PRELOAD_PRE_SHOOT_DELAY),
                                    new PrepareShootCommand(robot),
                                    new WaitCommand(PRE_SHOOT_DELAY)
                            ),
                            new GoToRestingStateCommand(robot)
                    ),
                    new WaitCommand(SHOOT_DELAY),
                    new ShootThreeBallsCommand(robot)
            );
            lastPose = getShootingPose(shootingPos);
        }

        ParkSettings parkSetting = prompter.get("park");
        if (parkSetting != null && parkSetting != ParkSettings.NO_PARK) {
            Pose parkPose = getParkPose();
            if (parkSetting == ParkSettings.FAR_PARK) {
                // TODO: replace with real FAR park pose when known
                parkPose = getParkPose();
            }
            PathChain parkPath = robot.follower
                    .pathBuilder()
                    .addPath(new BezierLine(lastPose, parkPose))
                    .setLinearHeadingInterpolation(lastPose.getHeading(), parkPose.getHeading())
                    .build();

            paths.addCommands(
                    new ParallelCommandGroup(
                            new GoToRestingStateCommand(robot),
                            new FollowPathCommand(robot.follower, parkPath, false)
                    )
            );
        }
    }

    public Pose getIntakePose(IntakePosition intakePosition) {
        Pose2d p;
        switch (intakePosition) {
            case POSITION_1:
                p = INTAKE_1_POSE;
                break;
            case POSITION_2:
                p = INTAKE_2_POSE;
                break;
            case POSITION_3:
                p = INTAKE_3_POSE;
                break;
            case HUMAN_PLAYER:
                throw new NotImplementedError("human pllayer not yet implemented");
            default:
                throw new IllegalArgumentException("invalid intake position");
        }
        return p.toPedro(mirror);
    }

    public Pose getPrepareIntakePose(IntakePosition intakePosition) {
        Pose2d p;
        switch (intakePosition) {
            case POSITION_1:
                p = PREPARE_INTAKE_1_POSE;
                break;
            case POSITION_2:
                p = PREPARE_INTAKE_2_POSE;
                break;
            case POSITION_3:
                p = PREPARE_INTAKE_3_POSE;
                break;
            case HUMAN_PLAYER:
                throw new NotImplementedError("human pllayer not yet implemented");
            default:
                throw new IllegalArgumentException("invalid intake position");
        }
        return p.toPedro(mirror);
    }

    public Pose getShootingPose(ShootingPosition shootingPosition) {
        Pose2d p;
        switch (shootingPosition) {
            case NEAR_TRIANGLE:
                p = SHOOT_POSE;
                break;
            case FAR_TRIANGLE:
                p = SHOOT_POSE; // TODO: change when far triangle pose is known
                break;
            default:
                throw new IllegalArgumentException("invalid shooting position");
        }
        return p.toPedro(mirror);
    }

    public Pose getParkPose() {
        return NEAR_PARK_POSE.toPedro(mirror);
    }

    public Pose getPushGatePose() {
        return PUSH_GATE_POSE.toPedro(mirror);
    }

    public Pose getShootPreloadPose() {
        return SHOOT_PRELOAD_POSE.toPedro(mirror);
    }

    public void runOpMode() {
        hardwareMap.dcMotor.get("motorFrontLeft").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardwareMap.dcMotor.get("motorFrontLeft").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL, RobotHardware.HardwareOptions.CAMERA);
        robot.init(hardware, telemetry);

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        prompter.prompt("alliance", new OptionPrompt<>("Select Alliance", Team.RED, Team.BLUE))
                .prompt("start", new OptionPrompt<>("Starting Position", StartingPosition.NEAR, StartingPosition.FAR))
                .prompt("shootPreload", new BooleanPrompt("Shoot Preload?", true))
                .prompt("intake1", new DetailedOptionPrompt<>(
                        "First Intake Position",
                        "(goal)",
                        "(human player)",
                        IntakePosition.POSITION_1,
                        IntakePosition.POSITION_2,
                        IntakePosition.POSITION_3
                ))
                .prompt("shootingPosition1", new OptionPrompt<>(
                        "Shooting Position After First Intake",
                        ShootingPosition.NEAR_TRIANGLE,
                        ShootingPosition.FAR_TRIANGLE
                ))
                .prompt("intake2", new DetailedOptionPrompt<>(
                        "Second Intake Position",
                        "(goal)",
                        "(human player)",
                        IntakePosition.POSITION_1,
                        IntakePosition.POSITION_2,
                        IntakePosition.POSITION_3
                ))
                .prompt("shootingPosition2", new OptionPrompt<>(
                        "Shooting Position After Second Intake",
                        ShootingPosition.NEAR_TRIANGLE,
                        ShootingPosition.FAR_TRIANGLE
                ))
                .prompt("intake3", new DetailedOptionPrompt<>(
                        "Third Intake Position",
                        "(goal)",
                        "(human player)",
                        IntakePosition.POSITION_1,
                        IntakePosition.POSITION_2,
                        IntakePosition.POSITION_3
                ))
                .prompt("shootingPosition3", new OptionPrompt<>(
                        "Shooting Position After Third Intake",
                        ShootingPosition.NEAR_TRIANGLE,
                        ShootingPosition.FAR_TRIANGLE
                ))
                .prompt("park", new OptionPrompt<>(
                        "Park at the End?",
                        ParkSettings.NEAR_PARK,
                        ParkSettings.FAR_PARK,
                        ParkSettings.NO_PARK
                ))
                .onComplete(this::onPromptsComplete);

        while (opModeInInit()) {
            prompter.run();
        }

        robot.follower.setMaxPower(MAX_POWER);

        // actionList was built in onPromptsComplete
        waitForStart();

        lastLoop = System.nanoTime();
        CommandScheduler.getInstance().schedule(paths);

        while (opModeIsActive()) {
            // Manually clear the bulk read cache. Deleting this would be catastrophic b/c stale
            // vals would be used.
            for (LynxModule hub : hardware.allHubs) {
                hub.clearBulkCache();
            }

            CommandScheduler.getInstance().run();

//            blackboard.put(MOTIF_DATA_KEY, robot.camera.getGlyph());
            blackboard.put(AUTO_ENDING_DATA_KEY, robot.follower.getPose());
            blackboard.put(SPINDEXER_POSITION_KEY, robot.spindexer.getPosition());

            hardware.write();

            FtcDashDrawing.drawDebug(robot.follower);
            long time = System.nanoTime();
            long dt = time - lastLoop;
            lastLoop = time;
            robot.telemetry.addData("Loop Time (ms)", String.format("%.2f", dt / 1e6));
            robot.telemetry.update();
        }
    }
}