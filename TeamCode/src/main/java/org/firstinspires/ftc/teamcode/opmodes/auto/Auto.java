package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.FieldConstants.AUTO_ENDING_DATA_KEY;
import static org.firstinspires.ftc.teamcode.FieldConstants.SPINDEXER_POSITION_KEY;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.pedroPathing.FixedHeadingInterpolator;
import org.firstinspires.ftc.teamcode.pedroPathing.FtcDashDrawing;
import org.firstinspires.ftc.teamcode.robot.command.intake.SetIntakePitchCommand;
import org.firstinspires.ftc.teamcode.robot.command.shooter.ShootThreeBallsCommand;
import org.firstinspires.ftc.teamcode.robot.command.shooter.ToggleAutoTurretCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.PrepareShootCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.WaitForSpindexerYawCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToIntakeStateCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToRestingStateCommand;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake.IntakePitch;
import org.firstinspires.ftc.teamcode.robot.subsystems.vision.CameraSubsystem;

import java.util.HashMap;
import java.util.Map;

@Config
public abstract class Auto extends LinearOpMode {
    public static Map<CameraSubsystem.GLYPH,CameraSubsystem.GLYPH> blueMotifMap = new HashMap<>();
    public static Map<CameraSubsystem.GLYPH,CameraSubsystem.GLYPH> redMotifMap = new HashMap<>();
    static {
        blueMotifMap.put(CameraSubsystem.GLYPH.PPG, CameraSubsystem.GLYPH.PGP);
        blueMotifMap.put(CameraSubsystem.GLYPH.PGP, CameraSubsystem.GLYPH.GPP);
        blueMotifMap.put(CameraSubsystem.GLYPH.GPP, CameraSubsystem.GLYPH.PPG);

        redMotifMap.put(CameraSubsystem.GLYPH.PPG, CameraSubsystem.GLYPH.GPP);
        redMotifMap.put(CameraSubsystem.GLYPH.GPP, CameraSubsystem.GLYPH.PGP);
        redMotifMap.put(CameraSubsystem.GLYPH.PGP, CameraSubsystem.GLYPH.PPG);
    }

    public static long TIME_UNTIL_START_SCANNING_GLYPHS = 200;

    public static boolean stopAfterPreload = false;

    public static double MAX_POWER = 1.0;

    public static Pose2d SHOOT_PRELOAD_POSE = new Pose2d(50.0, 104.644, Math.toRadians(315));
    public static Double SHOOT_PRELOAD_RPM = null;
    public static Pose2d SHOOT_EDGE_POSE = new Pose2d(50, 94, Math.toRadians(45));
    public static Pose2d SHOOT_LAST_POSE = new Pose2d(50, 114.644, Math.toRadians(315));

    public static Pose2d PREPARE_INTAKE_1_POSE = new Pose2d(52.598, 85.149, Math.toRadians(180));
    public static Pose2d INTAKE_1_POSE = new Pose2d(25, 85.149, Math.toRadians(180));
    public static Pose2d PUSH_GATE_POSE = new Pose2d(23, 72.827, Math.toRadians(180));

    public static Pose2d PREPARE_INTAKE_2_POSE = new Pose2d(PREPARE_INTAKE_1_POSE.x, 60, Math.toRadians(180));
    public static Pose2d INTAKE_2_POSE = new Pose2d(20, 62, Math.toRadians(180));

    public static Pose2d PREPARE_INTAKE_3_POSE = new Pose2d(PREPARE_INTAKE_1_POSE.x, 37, Math.toRadians(180));
    public static Pose2d INTAKE_3_POSE = new Pose2d(20, 39, Math.toRadians(180));

    public static Pose2d PARK_POSE = new Pose2d(52.282, 120.575, Math.toRadians(315));

    public static int PRE_INTAKE_DELAY = 0;
    public static int INTAKE_DELAY = 400;
    public static int PRELOAD_PRE_SHOOT_DELAY = 250;
    public static int PRE_SHOOT_DELAY = 0;
    public static int SHOOT_DELAY = 0;

    public static double MAX_DRIVETRAIN_POWER_INTAKING = 0.8;

    private final RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();
    private final Team team;

    private PathChain shootPreloadPath;
    private PathChain prepareIntake1Path, intake1Path, shoot1Path;
    private PathChain prepareIntake2Path, intake2Path, shoot2Path;
    private PathChain prepareIntake3Path, intake3Path, shoot3Path;
    private PathChain parkPath;

    private Command shootPreloadCommand;
    private Command intake1Command, shoot1Command;
    private Command intake2Command, shoot2Command;
    private Command intake3Command, shoot3Command;
    private Command parkCommand;
    double turretAngleForMotif;

    private long lastLoop = System.nanoTime();

    protected Auto(Team team) {

        this.team = team;
        if (team == Team.BLUE) {
            robot.goalPos = FieldConstants.BLUE_GOAL_POS;
        } else {
            robot.goalPos = FieldConstants.RED_GOAL_POS;
        }
    }

    private void buildPaths(Pose2d startPose, boolean mirror) {
        Pose shootPreloadPose = SHOOT_PRELOAD_POSE.toPedro();
        Pose prepareIntake1Pose = PREPARE_INTAKE_1_POSE.toPedro();
        Pose intake1Pose = INTAKE_1_POSE.toPedro();
        Pose pushGateControl = new Pose(44.87356321839081, 72.82758620689656);
        Pose pushGatePose = PUSH_GATE_POSE.toPedro();
        Pose shootEdgePose = SHOOT_EDGE_POSE.toPedro();
        Pose shootLastPose = SHOOT_LAST_POSE.toPedro();
        Pose prepareIntake2Pose = PREPARE_INTAKE_2_POSE.toPedro();
        Pose intake2Control = new Pose(56.751, 69.765);
        Pose intake2Pose = INTAKE_2_POSE.toPedro();
        Pose prepareIntake3Pose = PREPARE_INTAKE_3_POSE.toPedro();
        Pose intake3Control = new Pose(56.751, 45.668);
        Pose intake3Pose = INTAKE_3_POSE.toPedro();
        Pose parkPose = PARK_POSE.toPedro();

        if (mirror) {
            shootPreloadPose = shootPreloadPose.mirror();
            prepareIntake1Pose = prepareIntake1Pose.mirror();
            intake1Pose = intake1Pose.mirror();
            pushGateControl = pushGateControl.mirror();
            pushGatePose = pushGatePose.mirror();
            shootEdgePose = shootEdgePose.mirror();
            shootLastPose = shootLastPose.mirror();
            prepareIntake2Pose = prepareIntake2Pose.mirror();
            intake2Control = intake2Control.mirror();
            intake2Pose = intake2Pose.mirror();
            prepareIntake3Pose = prepareIntake3Pose.mirror();
            intake3Control = intake3Control.mirror();
            intake3Pose = intake3Pose.mirror();
            parkPose = parkPose.mirror();
        }

        Follower follower = robot.follower;
        shootPreloadPath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(startPose.toPedro(), shootPreloadPose)
                )
                .setLinearHeadingInterpolation(startPose.heading, shootPreloadPose.getHeading())
                .build();

        prepareIntake1Path = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(shootPreloadPose, prepareIntake1Pose)
                )
                .setLinearHeadingInterpolation(shootPreloadPose.getHeading(), prepareIntake1Pose.getHeading())
                .build();
        intake1Path = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(prepareIntake1Pose, intake1Pose)
                )
                .setLinearHeadingInterpolation(prepareIntake1Pose.getHeading(), intake1Pose.getHeading())
                .build();
        shoot1Path = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(intake1Pose, shootEdgePose)
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(0.0, 0.4, HeadingInterpolator.tangent),
                                new HeadingInterpolator.PiecewiseNode(0.4, 1.0,
                                        FixedHeadingInterpolator.linearFromPoint(() -> robot.follower.getHeading(), shootEdgePose.getHeading(), 0.4, 0.8)
                                )
                        )
                )
                .setReversed()
                .build();

        prepareIntake2Path = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                shootEdgePose,
                                intake2Control,
                                prepareIntake2Pose
                        )
                )
                .setLinearHeadingInterpolation(shoot1Path.getFinalHeadingGoal(), prepareIntake2Pose.getHeading())
                .build();
        intake2Path = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(prepareIntake2Pose, intake2Pose)
                )
                .setLinearHeadingInterpolation(prepareIntake2Pose.getHeading(), intake2Pose.getHeading())
                .build();
        shoot2Path = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(intake2Pose, shootEdgePose)
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        prepareIntake3Path = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                shootEdgePose,
                                intake3Control,
                                prepareIntake3Pose
                        )
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(0.0, 0.4, HeadingInterpolator.tangent),
                                new HeadingInterpolator.PiecewiseNode(0.4, 1.0,
                                        FixedHeadingInterpolator.linearFromPoint(() -> robot.follower.getHeading(), prepareIntake3Pose.getHeading(), 0.4, 0.7)
                                )
                        )
                )
                .build();
        intake3Path = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(prepareIntake3Pose, intake3Pose)
                )
                .setLinearHeadingInterpolation(prepareIntake3Pose.getHeading(), intake3Pose.getHeading())
                .build();
        shoot3Path = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(intake3Pose, shootLastPose)
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        parkPath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(shootLastPose, parkPose)
                )
                .setLinearHeadingInterpolation(shootLastPose.getHeading(), parkPose.getHeading())
                .build();
    }

    private void buildCommands() {
        shootPreloadCommand = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new PrepareShootCommand(robot, SHOOT_PRELOAD_RPM),
                        new FollowPathCommand(robot.follower, shootPreloadPath, true),
                        new ConditionalCommand(
                                new ToggleAutoTurretCommand(robot, false, turretAngleForMotif),
                                new InstantCommand(() -> {}),
                                () -> robot.camera.gameGlyph != null // this checs if the init thing wored
                        )
                ),
                new ToggleAutoTurretCommand(robot, true),
                new WaitCommand(PRELOAD_PRE_SHOOT_DELAY),
                new ShootThreeBallsCommand(robot),
                new WaitForSpindexerYawCommand(robot.spindexer).withTimeout(500),
                new InstantCommand(() -> robot.camera.stopScanningForGlyphs()),
                new WaitCommand(SHOOT_DELAY)
        );

        intake1Command = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, prepareIntake1Path, true, MAX_DRIVETRAIN_POWER_INTAKING),
                        new GoToIntakeStateCommand(robot)
                ),
                new WaitCommand(PRE_INTAKE_DELAY),
                new FollowPathCommand(robot.follower, intake1Path, true),
                new WaitCommand(INTAKE_DELAY)
        );
        shoot1Command = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, shoot1Path, true),
                        new WaitCommand(250).andThen(new PrepareShootCommand(robot, SHOOT_PRELOAD_RPM))
                ),
                new SetIntakePitchCommand(robot.intake, IntakePitch.UP),
                new WaitCommand(PRE_SHOOT_DELAY),
                new ShootThreeBallsCommand(robot),
                new WaitForSpindexerYawCommand(robot.spindexer).withTimeout(2000),
                new WaitCommand(SHOOT_DELAY)
        );

        intake2Command = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, prepareIntake2Path, true, MAX_DRIVETRAIN_POWER_INTAKING),
                        new GoToIntakeStateCommand(robot)
                ),
                new WaitCommand(PRE_INTAKE_DELAY),
                new FollowPathCommand(robot.follower, intake2Path, true),
                new WaitCommand(INTAKE_DELAY)
        );
        shoot2Command = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, shoot2Path, true),
                        new WaitCommand(250).andThen(new PrepareShootCommand(robot, SHOOT_PRELOAD_RPM))
                ),
                new WaitCommand(PRE_SHOOT_DELAY),
                new ShootThreeBallsCommand(robot),
                new WaitForSpindexerYawCommand(robot.spindexer).withTimeout(2000),
                new WaitCommand(SHOOT_DELAY)
        );

        intake3Command = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, prepareIntake3Path, true, MAX_DRIVETRAIN_POWER_INTAKING),
                        new GoToIntakeStateCommand(robot)
                ),
                new WaitCommand(PRE_INTAKE_DELAY),
                new FollowPathCommand(robot.follower, intake3Path, true),
                new WaitCommand(INTAKE_DELAY)
        );
        shoot3Command = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, shoot3Path, true),
                        new WaitCommand(250).andThen(new PrepareShootCommand(robot, SHOOT_PRELOAD_RPM))
                ),
                new WaitCommand(PRE_SHOOT_DELAY),
                new ShootThreeBallsCommand(robot),
                new WaitForSpindexerYawCommand(robot.spindexer).withTimeout(2000),
                new WaitCommand(SHOOT_DELAY)
        );

        parkCommand = new SequentialCommandGroup(
                new GoToRestingStateCommand(robot),
                new FollowPathCommand(robot.follower, parkPath, false)
        );
    }

    public void runOpMode() {
        hardwareMap.dcMotor.get("motorFrontLeft").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardwareMap.dcMotor.get("motorFrontLeft").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL, RobotHardware.HardwareOptions.CAMERA);

        robot.init(hardware, telemetry);
        robot.camera.stopScanningForGlyphs();

        this.turretAngleForMotif = Team.BLUE.equals(team) ? ShooterSubsystem.turretLowerBound : ShooterSubsystem.turretUpperBound;
        robot.goalPos = team.getGoalPos();
        robot.follower.setStartingPose(team.getStartPosNear().toPedro());

        buildPaths(team.getStartPosNear(), Team.RED.equals(team));
        buildCommands();
        robot.follower.setMaxPower(MAX_POWER);

        // todo note that this will mean we always sort, for 9 balls this is ok but for 12+ we want this to be only in certain cases
        // todo do the rules require that we do ths after init?
        robot.setAutoSort(true);

        robot.camera.startScanningForGlyphs();

        // we can't spin shooter in init bc it's illegal
        robot.shooter.isAutoVelOn = false;
        robot.shooter.setSpeed(0D);
        CommandScheduler.getInstance().schedule(new GoToRestingStateCommand(robot));
        CommandScheduler.getInstance().schedule(new ToggleAutoTurretCommand(robot, false, turretAngleForMotif));
        while (opModeInInit()) {
            for (LynxModule hub : hardware.allHubs) {
                hub.clearBulkCache();
            }

            CommandScheduler.getInstance().run();

            hardware.write();

            long time = System.nanoTime();
            long dt = time - lastLoop;
            lastLoop = time;
            robot.telemetry.addData("Loop Time (ms)", String.format("%.2f", dt / 1e6));
            robot.telemetry.update();
        }

        // we're going to see the wrong one
        if (robot.camera.gameGlyph != null) {
            robot.camera.stopScanningForGlyphs();
            robot.shooter.isAutoTurretOn = true;
            if (Team.RED.equals(team)) {
                robot.camera.setGlyph(redMotifMap.get(robot.camera.gameGlyph));
            } else {
                robot.camera.setGlyph(blueMotifMap.get(robot.camera.gameGlyph));
            }
        } else {
            robot.camera.startScanningForGlyphs();
        }

        waitForStart();
        robot.shooter.isAutoVelOn = true;

        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                shootPreloadCommand,
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                intake1Command, shoot1Command,
                                intake2Command, shoot2Command,
                                intake3Command, shoot3Command,
                                parkCommand
                        ),
                        new SequentialCommandGroup(
                                new GoToRestingStateCommand(robot)
                        ),
                        () -> !stopAfterPreload
                )
        ));

        lastLoop = System.nanoTime();

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