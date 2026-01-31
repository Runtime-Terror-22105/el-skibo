package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.FieldConstants.AUTO_ENDING_DATA_KEY;
import static org.firstinspires.ftc.teamcode.FieldConstants.MOTIF_DATA_KEY;
import static org.firstinspires.ftc.teamcode.FieldConstants.SPINDEXER_POSITION_KEY;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.AFTER_GATE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.GATE_CONTROL_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.GATE_INTAKE_TIMEOUT;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_1_CONTROL;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_1_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_2_CONTROL;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_2_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_3_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.INTAKE_DELAY;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.MAX_DRIVETRAIN_POWER_INTAKING;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.PRELOAD_PRE_SHOOT_DELAY;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.PREPARE_INTAKE_3_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.PRE_INTAKE_DELAY;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.PRE_SHOOT_DELAY;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.RELAXED_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_DELAY;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_EDGE_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_LAST_POSE;
import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_PRELOAD_POSE;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.pedroPathing.FixedHeadingInterpolator;
import org.firstinspires.ftc.teamcode.pedroPathing.FtcDashDrawing;
import org.firstinspires.ftc.teamcode.robot.auto.AutoBuilder;
import org.firstinspires.ftc.teamcode.robot.auto.PathUtil;
import org.firstinspires.ftc.teamcode.robot.command.WaitForIntakeCommand;
import org.firstinspires.ftc.teamcode.robot.command.shooter.ShootThreeBallsCommand;
import org.firstinspires.ftc.teamcode.robot.command.shooter.ToggleAutoTurretCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.PrepareShootCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.WaitForSpindexerYawCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToIntakeStateCommand;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.subsystems.vision.CameraSubsystem;

import java.util.HashMap;
import java.util.Map;


@Config
@Autonomous(name = "One Auto To Rule Them All", group = "Auto")
public class OneAutoToRuleThemAll extends LinearOpMode {
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

    public static double MAX_POWER = 1.0;

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

    protected OneAutoToRuleThemAll(Team team) {
        this.team = team;
        robot.goalPos = team.getGoalPos();
        robot.color = team;
    }

    private void buildPaths(Pose2d startPose, boolean mirror) {
        Pose shootPreloadPose = SHOOT_PRELOAD_POSE.toPedro();
        Pose shootEdgePose = SHOOT_EDGE_POSE.toPedro();
        Pose shootLastPose = SHOOT_LAST_POSE.toPedro();
        Pose prepareIntake3Pose = PREPARE_INTAKE_3_POSE.toPedro();
        Pose intake3Control = new Pose(56.751, 45.668);
        Pose intake3Pose = INTAKE_3_POSE.toPedro();

        if (mirror) {
            shootPreloadPose = shootPreloadPose.mirror();
            shootEdgePose = shootEdgePose.mirror();
            shootLastPose = shootLastPose.mirror();
            prepareIntake3Pose = prepareIntake3Pose.mirror();
            intake3Control = intake3Control.mirror();
            intake3Pose = intake3Pose.mirror();
        }

        Follower follower = robot.follower;
        shootPreloadPath = PathUtil.addPathBuilderLine(robot, startPose, new Pose2d(shootPreloadPose), false, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
//                .setNoDeceleration()
                .build();

//        prepareIntake1Path = PathUtil.createLinePath(robot, shootPreloadPath, PREPARE_INTAKE_2_POSE, mirror, false, false);
        intake1Path = PathUtil.addPathBuilderCurve(robot, shootPreloadPath, INTAKE_2_CONTROL, INTAKE_2_POSE, mirror, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
//        pushGatePath = createLinePath(intake1Path, PUSH_GATE_POSE, mirror, false, false);
        shoot1Path = PathUtil.addPathBuilderLine(robot, intake1Path, SHOOT_EDGE_POSE, mirror, true, true)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
//                .setNoDeceleration()
                .build();

        // TODO: for the second gate intake, this heading will not be correct
//        prepareGatePath = PathUtil.createCurvePath(robot, shoot1Path, GATE_CONTROL_POSE, BEFORE_GATE, mirror, false, false);
        hitGatePath = PathUtil.createCurvePath(robot, shoot1Path, GATE_CONTROL_POSE, AFTER_GATE, mirror, false, false);
        gateToShootPath = PathUtil.addPathBuilderLine(robot, hitGatePath, SHOOT_EDGE_POSE, mirror, true, true)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
//                .setNoDeceleration()
                .build();

//        prepareIntake2Path = createCurvePath(SHOOT_EDGE_POSE, INTAKE_2_CONTROL, PREPARE_INTAKE_2_POSE, mirror, false);
//        prepareIntake2Path = PathUtil.createLinePath(robot, gateToShootPath, PREPARE_INTAKE_1_POSE, mirror, false, false);
        intake2Path = PathUtil.addPathBuilderCurve(robot, gateToShootPath, INTAKE_1_CONTROL, INTAKE_1_POSE, mirror, false, false)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
                .build();
        shoot2Path = PathUtil.addPathBuilderLine(robot, intake2Path, SHOOT_LAST_POSE, mirror, true, true)
                .setConstraintsForLast(RELAXED_CONSTRAINTS)
//                .setNoDeceleration()
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
    }

    private void buildCommands() {
        shootPreloadCommand = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new PrepareShootCommand(robot),
                        new FollowPathCommand(robot.follower, shootPreloadPath, false)
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
                        new FollowPathCommand(robot.follower, shoot1Path, false),
                        new WaitCommand(250).andThen(new PrepareShootCommand(robot))
                ),
                new WaitCommand(PRE_SHOOT_DELAY),
                new ShootThreeBallsCommand(robot),
                new WaitForSpindexerYawCommand(robot.spindexer).withTimeout(2000),
                new WaitCommand(SHOOT_DELAY)
        );

        intake2Command = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, intake2Path, true, MAX_DRIVETRAIN_POWER_INTAKING),
                        new GoToIntakeStateCommand(robot)
                ),
//                new WaitCommand(PRE_INTAKE_DELAY),
//                new FollowPathCommand(robot.follower, intake2Path, true),
                new WaitCommand(INTAKE_DELAY)
        );
        shoot2Command = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, shoot2Path, false),
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
                new WaitForIntakeCommand(robot).withTimeout(GATE_INTAKE_TIMEOUT)
        );
        shootGateCommand = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, gateToShootPath, false),
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
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL, RobotHardware.HardwareOptions.CAMERA);

        robot.init(hardware, telemetry);
        robot.camera.stopScanningForGlyphs();

        this.turretAngleForMotif = Math.PI + (Team.BLUE.equals(team) ? -1 : 1) * Math.toRadians(30);

        // TODO: the autobuildder class currently does not handle the init logic
        // and assumes starting near side at the moment for preload
        AutoBuilder auto = new AutoBuilder(robot, this.team, AutoBuilder.StartingConfiguration.NEAR);
        robot.follower.setMaxPower(MAX_POWER);

        // todo note that this will mean we always sort, for 9 balls this is ok but for 12+ we want this to be only in certain cases
        // todo do the rules require that we do ths after init?
        robot.setAutoSort(false);

        robot.camera.startScanningForGlyphs();

        // we can't spin shooter in init bc it's illegal
        robot.shooter.isAutoVelOn = false;
        robot.shooter.setSpeed(0D);
        CommandScheduler.getInstance().schedule(new ToggleAutoTurretCommand(robot, false, turretAngleForMotif));
        while (opModeInInit()) {
            for (LynxModule hub : hardware.allHubs) {
                hub.clearBulkCache();
            }

            CommandScheduler.getInstance().run();

            robot.write();

            // region poem
            // Three Autos for the Cadding-kings under the sky,
            // Seven for the code-lords in their halls of stone,
            // Nine for Building Men doomed to die,
            // One for the Wire Lord on his wiring throne,
            // In the Land of Winecreek where the Terrors lie.
            // One Auto to rule them all, One Auto to find them,
            // One Auto to bring them all and in the darkness bind them
            // In the Land of Winecreek where the Terrors lie.
            robot.telemetry.addLine("Three Autos for the Cadding-kings under the sky,");
            robot.telemetry.addLine("Seven for the code-lords in their halls of stone,");
            robot.telemetry.addLine("Nine for Building Men doomed to die,");
            robot.telemetry.addLine("One for the Wire Lord on his wiring throne,");
            robot.telemetry.addLine("In the Land of Winecreek where the Terrors lie.");
            robot.telemetry.addLine("One Auto to rule them all, One Auto to find them,");
            robot.telemetry.addLine("One Auto to bring them all and in the darkness bind them");
            robot.telemetry.addLine("In the Land of Winecreek where the Terrors lie.");
            robot.telemetry.update();
            // endregion poem
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
                auto.shootPreload(),
                auto.spikeIntake(2), auto.spikeShoot(2),
                auto.intakeGate(), auto.shootGate(),
                auto.intakeGate(), auto.shootGate(),
                auto.spikeIntake(1), auto.spikeShoot(1)
        ));

        lastLoop = System.nanoTime();

        while (opModeIsActive()) {
            // Manually clear the bulk read cache. Deleting this would be catastrophic b/c stale
            // vals would be used.
            for (LynxModule hub : hardware.allHubs) {
                hub.clearBulkCache();
            }

            CommandScheduler.getInstance().run();

            blackboard.put(MOTIF_DATA_KEY, robot.camera.getGlyph());
            blackboard.put(AUTO_ENDING_DATA_KEY, robot.follower.getPose());
            blackboard.put(SPINDEXER_POSITION_KEY, robot.spindexer.getPosition());

            hardware.write();

            // region debugginig
            FtcDashDrawing.drawDebug(robot.follower);
            long time = System.nanoTime();
            long dt = time - lastLoop;
            lastLoop = time;
            robot.telemetry.addData("Loop Time (ms)", String.format("%.2f", dt / 1e6));
            if (!auto.hasFinished()) {
                duration = System.currentTimeMillis() - startTime;
            }
            robot.telemetry.addData("Auto Time (s)", String.format("%.2f", (System.currentTimeMillis() - startTime) / 1000.0));
            robot.telemetry.addData("Auto Duration (s)", String.format("%.2f", duration / 1000.0));
            robot.telemetry.update();
            // endregion
        }
    }
}
