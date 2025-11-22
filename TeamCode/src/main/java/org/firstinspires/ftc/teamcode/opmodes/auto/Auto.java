package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.PoseConverter;
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
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;
import org.firstinspires.ftc.teamcode.pedroPathing.FtcDashDrawing;
import org.firstinspires.ftc.teamcode.robot.command.shooter.SetShooterRPMCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.TransferCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToIntakeStateCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToRestingStateCommand;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@Config
@Configurable
public abstract class Auto extends LinearOpMode {
    public static double MAX_POWER = 0.5;
    public static Pose2d SHOOT_PRELOAD_POSE = new Pose2d(50.0, 104.644, Math.toRadians(315));
    public static double SHOOT_PRELOAD_RPM = 0;
    public static Pose2d PREPARE_INTAKE_1_POSE = new Pose2d(52.598, 85.149, Math.toRadians(180));
    public static Pose2d INTAKE_1_POSE = new Pose2d(30.2, 85.149, Math.toRadians(180));
    public static Pose2d PUSH_GATE_POSE = new Pose2d(44.873, 72.827, Math.toRadians(180));

    private final RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();
    private final Team team;

    private PathChain shootPreloadPath;
    private PathChain prepareIntake1Path;
    private PathChain intake1Path;
    private PathChain pushGate1Path;

    private Command shootPreloadCommand;
    private Command intake1Command;

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
    }

    private void buildCommands() {
        shootPreloadCommand = new SequentialCommandGroup(
                new GoToRestingStateCommand(robot),
                new SetShooterRPMCommand(robot.shooter, SHOOT_PRELOAD_RPM),
                new WaitCommand(500),
                new FollowPathCommand(robot.follower, shootPreloadPath, true),
                new WaitCommand(500),
//                new TransferCommand(robot, SHOOT_PRELOAD_RPM),
                new WaitCommand(500)
        );

        intake1Command = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, prepareIntake1Path, true),
                        new GoToIntakeStateCommand(robot)
                ),
                new WaitCommand(500),
                new FollowPathCommand(robot.follower, intake1Path, true),
                new WaitCommand(500),
                new FollowPathCommand(robot.follower, pushGate1Path, true)
        );
    }

    public void runOpMode() {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL);

        robot.init(hardware, telemetry);
        robot.goalPos = team.getGoalPos();
        robot.follower.setStartingPose(team.getStartPosAuto());

        buildPaths(team.getStartPosAuto());
        buildCommands();
        robot.follower.setMaxPower(MAX_POWER);

        waitForStart();

        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                shootPreloadCommand,
                intake1Command
        ));

        lastLoop = System.nanoTime();

        while (opModeIsActive()) {
            // Manually clear the bulk read cache. Deleting this would be catastrophic b/c stale
            // vals would be used.
            for (LynxModule hub : hardware.allHubs) {
                hub.clearBulkCache();
            }

            CommandScheduler.getInstance().run();

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
