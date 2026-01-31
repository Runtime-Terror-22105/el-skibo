package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.FieldConstants.AUTO_ENDING_DATA_KEY;
import static org.firstinspires.ftc.teamcode.FieldConstants.MOTIF_DATA_KEY;
import static org.firstinspires.ftc.teamcode.FieldConstants.SPINDEXER_POSITION_KEY;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.pedroPathing.FixedHeadingInterpolator;
import org.firstinspires.ftc.teamcode.pedroPathing.FtcDashDrawing;
import org.firstinspires.ftc.teamcode.robot.command.WaitForIntakeCommand;
import org.firstinspires.ftc.teamcode.robot.command.shooter.ShootThreeBallsCommand;
import org.firstinspires.ftc.teamcode.robot.command.shooter.ToggleAutoTurretCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.PrepareShootCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.WaitForSpindexerYawCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToIntakeStateCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToRestingStateCommand;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.subsystems.vision.CameraSubsystem;
import org.firstinspires.ftc.teamcode.util.Profiler;

import java.util.HashMap;
import java.util.Map;


// Gate spam auto
@Config
public abstract class AutoSpam extends LinearOpMode {
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

    public static long HITTING_GATE_TIMEOUT = 1300;

    public static boolean stopAfterPreload = false;

    public static double MAX_POWER = 1.0;

    public static Pose2d SHOOT_PRELOAD_POSE = new Pose2d(50.0, 93, Math.toRadians(225));
    public static Pose2d SHOOT_EDGE_POSE = new Pose2d(60, 83, Math.toRadians(45));
    public static Pose2d SHOOT_LAST_POSE = new Pose2d(50, 116, Math.toRadians(315));

    public static Pose2d PREPARE_INTAKE_1_POSE = new Pose2d(52.598, 85.149, Math.toRadians(180));
    public static Pose2d INTAKE_1_POSE = new Pose2d(25, 85.149, Math.toRadians(180));

    public static Pose2d PREPARE_INTAKE_2_POSE = new Pose2d(PREPARE_INTAKE_1_POSE.x, 60, Math.toRadians(180));
    public static Pose2d INTAKE_2_CONTROL = new Pose2d(58, 58, 0);
    public static Pose2d INTAKE_2_POSE = new Pose2d(20, 60, Math.toRadians(180));

    public static Pose2d PREPARE_INTAKE_3_POSE = new Pose2d(PREPARE_INTAKE_1_POSE.x, 37, Math.toRadians(180));
    public static Pose2d INTAKE_3_POSE = new Pose2d(20, 39, Math.toRadians(180));

    public static Pose2d GATE_CONTROL_POSE = new Pose2d(55, 61, Math.toRadians(180));
    public static Pose2d BEFORE_GATE = new Pose2d(22.542, 62.2, Math.toRadians(155));
    public static Pose2d AFTER_GATE = new Pose2d(11, 62.2, Math.toRadians(155));

    public static int PRE_INTAKE_DELAY = 0;
    public static int INTAKE_DELAY = 600;
    public static int GATE_INTAKE_DELAY = 2000;
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

    private Command shootPreloadCommand;
    private Command intake1Command, shoot1Command;
    private Command intakeGateCommand, shootGateCommand;
    private Command intake2Command, shoot2Command;
    private Command intake3Command, shoot3Command;

    //    private PathChain pushGatePath;
    private PathChain prepareGatePath;
    private PathChain hitGatePath;
    private PathChain gateToShootPath;

    double turretAngleForMotif;

    private boolean hasFinished = false;
    private long duration = 0;
    private long startTime = 0;
    private long lastLoop = System.nanoTime();

    protected AutoSpam(Team team) {
        this.team = team;
        robot.goalPos = team.getGoalPos();
        robot.color = team;
    }

    private void buildPaths(Pose2d startPose, boolean mirror) {
        Pose shootEdgePose = SHOOT_EDGE_POSE.toPedro();
        Pose shootLastPose = SHOOT_LAST_POSE.toPedro();
        Pose prepareIntake3Pose = PREPARE_INTAKE_3_POSE.toPedro();
        Pose intake3Control = new Pose(56.751, 45.668);
        Pose intake3Pose = INTAKE_3_POSE.toPedro();

        if (mirror) {
            shootEdgePose = shootEdgePose.mirror();
            shootLastPose = shootLastPose.mirror();
            prepareIntake3Pose = prepareIntake3Pose.mirror();
            intake3Control = intake3Control.mirror();
            intake3Pose = intake3Pose.mirror();
        }

        Follower follower = robot.follower;
        shootPreloadPath = PathUtil.createLinePath(robot, startPose, SHOOT_PRELOAD_POSE, mirror, false, false);

//        prepareIntake1Path = PathUtil.createLinePath(robot, shootPreloadPath, PREPARE_INTAKE_2_POSE, mirror, false, false);
        intake1Path = PathUtil.createCurvePath(robot, shootPreloadPath, INTAKE_2_CONTROL, INTAKE_2_POSE, mirror, false, false);
//        pushGatePath = createLinePath(intake1Path, PUSH_GATE_POSE, mirror, false, false);
        shoot1Path = PathUtil.createLinePath(robot, intake1Path, SHOOT_EDGE_POSE, mirror, true, true);

        // TODO: for the second gate intake, this heading will not be correct
//        prepareGatePath = PathUtil.createCurvePath(robot, shoot1Path, GATE_CONTROL_POSE, BEFORE_GATE, mirror, false, false);
        hitGatePath = PathUtil.createCurvePath(robot, shoot1Path, GATE_CONTROL_POSE, AFTER_GATE, mirror, false, false);
        gateToShootPath = PathUtil.createLinePath(robot, hitGatePath, SHOOT_EDGE_POSE, mirror, true, true);

//        prepareIntake2Path = createCurvePath(SHOOT_EDGE_POSE, INTAKE_2_CONTROL, PREPARE_INTAKE_2_POSE, mirror, false);
        prepareIntake2Path = PathUtil.createLinePath(robot, gateToShootPath, PREPARE_INTAKE_1_POSE, mirror, false, false);
        intake2Path = PathUtil.createLinePath(robot, prepareIntake2Path, INTAKE_1_POSE, mirror, false, false);
        shoot2Path = PathUtil.createLinePath(robot, intake2Path, SHOOT_LAST_POSE, mirror, true, true);

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
    }

    private void buildCommands() {
        shootPreloadCommand = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new PrepareShootCommand(robot),
                        new FollowPathCommand(robot.follower, shootPreloadPath, true)
                ),
                new WaitCommand(PRELOAD_PRE_SHOOT_DELAY),
                new ShootThreeBallsCommand(robot),
                new WaitForSpindexerYawCommand(robot.spindexer).withTimeout(500),
                new InstantCommand(() -> robot.camera.stopScanningForGlyphs()),
                new WaitCommand(SHOOT_DELAY)
        );

        intake1Command = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, intake1Path, true, MAX_DRIVETRAIN_POWER_INTAKING),
                        new GoToIntakeStateCommand(robot)
                ),
//                new WaitCommand(PRE_INTAKE_DELAY),
//                new FollowPathCommand(robot.follower, intake1Path, true),
//                new FollowPathCommand(robot.follower, pushGatePath, true)
                new WaitForIntakeCommand(robot).withTimeout(INTAKE_DELAY)
        );
        shoot1Command = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, shoot1Path, true),
                        new WaitCommand(250).andThen(new PrepareShootCommand(robot))
                ),
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
                        new WaitCommand(250).andThen(new PrepareShootCommand(robot))
                ),
                new WaitCommand(PRE_SHOOT_DELAY),
                new ShootThreeBallsCommand(robot),
                new WaitForSpindexerYawCommand(robot.spindexer).withTimeout(2000),
                new WaitCommand(SHOOT_DELAY)
        );

        intakeGateCommand = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, hitGatePath, true, MAX_DRIVETRAIN_POWER_INTAKING),
                        new GoToIntakeStateCommand(robot)
                ),
//                new WaitCommand(PRE_INTAKE_DELAY),
//                new ParallelRaceGroup(
//                        new FollowPathCommand(robot.follower, hitGatePath, true),
//                        new WaitCommand(HITTING_GATE_TIMEOUT)
//                ),
                new WaitForIntakeCommand(robot).withTimeout(GATE_INTAKE_DELAY)
        );
        shootGateCommand = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, gateToShootPath, true),
                        new WaitCommand(250).andThen(new PrepareShootCommand(robot, true))
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
                        new WaitCommand(250).andThen(new PrepareShootCommand(robot))
                ),
                new WaitCommand(PRE_SHOOT_DELAY),
                new ShootThreeBallsCommand(robot),
                new WaitForSpindexerYawCommand(robot.spindexer).withTimeout(2000),
                new WaitCommand(SHOOT_DELAY)
        );
    }

    public void runOpMode() {
        Profiler.init();

        hardwareMap.dcMotor.get("motorFrontLeft").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardwareMap.dcMotor.get("motorFrontLeft").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL, RobotHardware.HardwareOptions.CAMERA);

        robot.init(hardware, telemetry);
        robot.camera.stopScanningForGlyphs();

        this.turretAngleForMotif = Math.PI + (Team.BLUE.equals(team) ? -1 : 1) * Math.toRadians(30);
        robot.goalPos = team.getGoalPos();
        robot.follower.setStartingPose(team.getStartPosNear().toPedro());

        buildPaths(team.getStartPosNear(), Team.RED.equals(team));
        buildCommands();
        robot.follower.setMaxPower(MAX_POWER);

        // todo note that this will mean we always sort, for 9 balls this is ok but for 12+ we want this to be only in certain cases
        // todo do the rules require that we do ths after init?
        robot.setAutoSort(false);

        robot.camera.startScanningForGlyphs();

        // we can't spin shooter in init bc it's illegal
        robot.shooter.isAutoVelOn = false;
        robot.shooter.setSpeed(0D);
//        CommandScheduler.getInstance().schedule(new GoToRestingStateCommand(robot));
        CommandScheduler.getInstance().schedule(new ToggleAutoTurretCommand(robot, false, turretAngleForMotif));
        while (opModeInInit()) {
            for (LynxModule hub : hardware.allHubs) {
                hub.clearBulkCache();
            }

            CommandScheduler.getInstance().run();

            robot.write();
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
        }

        waitForStart();
        robot.shooter.isAutoVelOn = true;
        robot.shooter.isAutoAimOn = true;
        robot.shooter.alwaysUpdateTurret = true;
        startTime = System.currentTimeMillis();

        CommandScheduler.getInstance().schedule(new ToggleAutoTurretCommand(robot, true));
        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                shootPreloadCommand,
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                intake1Command, shoot1Command,
                                intakeGateCommand, shootGateCommand,
                                intakeGateCommand, shootGateCommand,
//                                intakeGateCommand, shootGateCommand,
                                intake2Command, shoot2Command
                        ),
                        new SequentialCommandGroup(
                                new GoToRestingStateCommand(robot)
                        ),
                        () -> !stopAfterPreload
                ),
                new InstantCommand(() -> hasFinished = true)
        ));

        lastLoop = System.nanoTime();

        while (opModeIsActive()) {
            Profiler.start();

            // Manually clear the bulk read cache. Deleting this would be catastrophic b/c stale
            // vals would be used.
            Profiler.push("clear_cache");
            for (LynxModule hub : hardware.allHubs) {
                hub.clearBulkCache();
            }
            Profiler.pop();

            Profiler.push("commands");
            CommandScheduler.getInstance().run();
            Profiler.pop();

            blackboard.put(MOTIF_DATA_KEY, robot.camera.getGlyph());
            blackboard.put(AUTO_ENDING_DATA_KEY, robot.follower.getPose());
            blackboard.put(SPINDEXER_POSITION_KEY, robot.spindexer.getPosition());

            Profiler.push("hwWrite");
            hardware.write();
            Profiler.pop();

            Profiler.push("debug");
            FtcDashDrawing.drawDebug(robot.follower);
            long time = System.nanoTime();
            long dt = time - lastLoop;
            lastLoop = time;
            robot.telemetry.addData("Loop Time (ms)", String.format("%.2f", dt / 1e6));
            if (!hasFinished) {
                duration = System.currentTimeMillis() - startTime;
            }
            robot.telemetry.addData("Auto Time (s)", String.format("%.2f", (System.currentTimeMillis() - startTime) / 1000.0));
            robot.telemetry.addData("Auto Duration (s)", String.format("%.2f", duration / 1000.0));
            robot.telemetry.update();
            Profiler.pop();

            Profiler.end();
            Profiler.sendFlamegraph(robot.telemetry);
        }
    }
}