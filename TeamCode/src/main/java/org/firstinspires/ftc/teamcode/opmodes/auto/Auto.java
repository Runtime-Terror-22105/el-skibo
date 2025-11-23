package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.FieldConstants.AUTO_ENDING_DATA_KEY;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

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
import org.firstinspires.ftc.teamcode.robot.subsystems.intake.IntakePitch;

@Config
@Configurable
public abstract class Auto extends LinearOpMode {
    public static double MAX_POWER = 1.0;

    public static Pose2d SHOOT_PRELOAD_POSE = new Pose2d(50.0, 104.644, Math.toRadians(315));
    public static double SHOOT_PRELOAD_RPM = 3500;

    public static Pose2d PREPARE_INTAKE_1_POSE = new Pose2d(52.598, 85.149, Math.toRadians(180));
    public static Pose2d INTAKE_1_POSE = new Pose2d(26, 85.149, Math.toRadians(210));
    public static Pose2d PUSH_GATE_POSE = new Pose2d(23, 72.827, Math.toRadians(180));
    public static Pose2d SHOOT_POSE = new Pose2d(60, 87.449, Math.toRadians(315));

    public static Pose2d PREPARE_INTAKE_2_POSE = new Pose2d(PREPARE_INTAKE_1_POSE.x, 63, Math.toRadians(180));
    public static Pose2d INTAKE_2_POSE = new Pose2d(INTAKE_1_POSE.x, 63, Math.toRadians(210));

    public static Pose2d PREPARE_INTAKE_3_POSE = new Pose2d(PREPARE_INTAKE_1_POSE.x, 40, Math.toRadians(180));
    public static Pose2d INTAKE_3_POSE = new Pose2d(24, 40, Math.toRadians(210));

    public static Pose2d PARK_POSE = new Pose2d(52.282, 120.575, Math.toRadians(315));

    public static int PRE_INTAKE_DELAY = 0;
    public static int INTAKE_DELAY = 0;
    public static int PRE_SHOOT_DELAY = 0;
    public static int SHOOT_DELAY = 0;

    private final RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();
    private final Team team;

    private PathChain shootPreloadPath;
    private PathChain prepareIntake1Path, intake1Path, pushGate1Path, shoot1Path;
    private PathChain prepareIntake2Path, intake2Path, shoot2Path;
    private PathChain prepareIntake3Path, intake3Path, shoot3Path;
    private PathChain parkPath;

    private Command shootPreloadCommand;
    private Command intake1Command, shoot1Command;
    private Command intake2Command, shoot2Command;
    private Command intake3Command, shoot3Command;
    private Command parkCommand;

    private long lastLoop = System.nanoTime();

    protected Auto(Team team) {
        this.team = team;
    }

    private void buildPaths(Pose2d startPose) {
        Follower follower = robot.follower;
        shootPreloadPath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(startPose.toPedro(), SHOOT_PRELOAD_POSE.toPedro())
                )
                .setLinearHeadingInterpolation(startPose.heading, SHOOT_PRELOAD_POSE.heading)
                .build();

        prepareIntake1Path = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(SHOOT_PRELOAD_POSE.toPedro(), PREPARE_INTAKE_1_POSE.toPedro())
                )
                .setLinearHeadingInterpolation(SHOOT_PRELOAD_POSE.heading, PREPARE_INTAKE_1_POSE.heading)
                .build();
        intake1Path = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(PREPARE_INTAKE_1_POSE.toPedro(), INTAKE_1_POSE.toPedro())
                )
                .setLinearHeadingInterpolation(PREPARE_INTAKE_1_POSE.heading, INTAKE_1_POSE.heading)
                .build();
        pushGate1Path = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(INTAKE_1_POSE.toPedro(),
                                new Pose(44.87356321839081, 72.82758620689656),
                                PUSH_GATE_POSE.toPedro())
                )
                .setLinearHeadingInterpolation(INTAKE_1_POSE.heading, PUSH_GATE_POSE.heading)
                .build();
        shoot1Path = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(PUSH_GATE_POSE.toPedro(), SHOOT_POSE.toPedro())
                )
                .setLinearHeadingInterpolation(PUSH_GATE_POSE.heading, SHOOT_POSE.heading)
                .build();

        prepareIntake2Path = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                SHOOT_POSE.toPedro(),
                                new Pose(55.780, 69.765),
                                PREPARE_INTAKE_2_POSE.toPedro()
                        )
                )
                .setLinearHeadingInterpolation(SHOOT_POSE.heading, PREPARE_INTAKE_2_POSE.heading)
                .build();
        intake2Path = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(PREPARE_INTAKE_2_POSE.toPedro(), INTAKE_2_POSE.toPedro())
                )
                .setLinearHeadingInterpolation(PREPARE_INTAKE_2_POSE.heading, INTAKE_2_POSE.heading)
                .build();
        shoot2Path = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(INTAKE_2_POSE.toPedro(), SHOOT_POSE.toPedro())
                )
                .setLinearHeadingInterpolation(INTAKE_2_POSE.heading, SHOOT_POSE.heading)
                .build();

        prepareIntake3Path = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                SHOOT_POSE.toPedro(),
                                new Pose(56.751, 45.668),
                                PREPARE_INTAKE_3_POSE.toPedro()
                        )
                )
                .setLinearHeadingInterpolation(SHOOT_POSE.heading, PREPARE_INTAKE_3_POSE.heading)
                .build();
        intake3Path = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(PREPARE_INTAKE_3_POSE.toPedro(), INTAKE_3_POSE.toPedro())
                )
                .setLinearHeadingInterpolation(PREPARE_INTAKE_3_POSE.heading, INTAKE_3_POSE.heading)
                .build();
        shoot3Path = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(INTAKE_3_POSE.toPedro(), SHOOT_POSE.toPedro())
                )
                .setLinearHeadingInterpolation(INTAKE_3_POSE.heading, SHOOT_POSE.heading)
                .build();

        parkPath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(SHOOT_POSE.toPedro(), PARK_POSE.toPedro())
                )
                .setLinearHeadingInterpolation(SHOOT_POSE.heading, PARK_POSE.heading)
                .build();
    }

    private void buildCommands() {
        shootPreloadCommand = new SequentialCommandGroup(
                new ParallelCommandGroup(
//                        new PrepareShootCommand(robot, SHOOT_PRELOAD_RPM),
                        new FollowPathCommand(robot.follower, shootPreloadPath, true)
                ),
                new WaitCommand(PRE_SHOOT_DELAY),
                new ShootThreeBallsCommand(robot),
                new WaitForSpindexerYawCommand(robot.spindexer).withTimeout(2000),
                new WaitCommand(SHOOT_DELAY)
        );

        intake1Command = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, prepareIntake1Path, true),
                        new GoToIntakeStateCommand(robot)
                ),
                new WaitCommand(PRE_INTAKE_DELAY),
                new FollowPathCommand(robot.follower, intake1Path, true),
                new WaitCommand(INTAKE_DELAY),
                new ParallelCommandGroup(
                        new WaitCommand(250).andThen(new PrepareShootCommand(robot, SHOOT_PRELOAD_RPM, IntakePitch.DOWN)),
                        new FollowPathCommand(robot.follower, pushGate1Path, true, 0.5)
                )
        );
        shoot1Command = new SequentialCommandGroup(
                new FollowPathCommand(robot.follower, shoot1Path, true),
                new SetIntakePitchCommand(robot.intake, IntakePitch.UP),
                new WaitCommand(PRE_SHOOT_DELAY),
                new ShootThreeBallsCommand(robot),
                new WaitForSpindexerYawCommand(robot.spindexer).withTimeout(2000),
                new WaitCommand(SHOOT_DELAY)
        );

        intake2Command = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, prepareIntake2Path, true),
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
                        new FollowPathCommand(robot.follower, prepareIntake3Path, true),
                        new GoToIntakeStateCommand(robot)
                ),
                new WaitCommand(PRE_INTAKE_DELAY),
                new FollowPathCommand(robot.follower, intake3Path, true),
                new WaitCommand(INTAKE_DELAY),
                new PrepareShootCommand(robot, SHOOT_PRELOAD_RPM)
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
                new FollowPathCommand(robot.follower, parkPath, true)
        );
    }

    public void runOpMode() {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL);

        robot.init(hardware, telemetry);
        robot.goalPos = team.getGoalPos();
        robot.follower.setStartingPose(team.getStartPosAuto().toPedro());

        buildPaths(team.getStartPosAuto());
        buildCommands();
        robot.follower.setMaxPower(MAX_POWER);
        robot.shooter.isAutoVelOn = false;

        CommandScheduler.getInstance().schedule(new PrepareShootCommand(robot, SHOOT_PRELOAD_RPM));
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


        waitForStart();

        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                shootPreloadCommand,
                intake1Command, shoot1Command,
                intake2Command, shoot2Command,
                intake3Command, shoot3Command,
                parkCommand
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
