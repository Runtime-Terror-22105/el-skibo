package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.FieldConstants.AUTO_ENDING_DATA_KEY;
import static org.firstinspires.ftc.teamcode.FieldConstants.SPINDEXER_POSITION_KEY;
import static org.firstinspires.ftc.teamcode.opmodes.auto.Auto.SHOOT_PRELOAD_POSE;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.pedroPathing.FtcDashDrawing;
import org.firstinspires.ftc.teamcode.robot.command.shooter.ShimmyShoot3Ball;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.PrepareShootCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.WaitForSpindexerYawCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToRestingStateCommand;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

public class LeaveAutoNear extends LinearOpMode {

    private final RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();
    private final Team team;


    private PathChain parkPath;
    private PathChain shootPreloadPath;

    private Command shootPreloadCommand;
    private Command parkCommand;

    private int PRELOAD_PRE_SHOOT_DELAY = 5000;


    protected LeaveAutoNear(Team team) {

        this.team = team;
        if (team == Team.BLUE){
            robot.goalPos = FieldConstants.BLUE_GOAL_POS;
        }
        else {
            robot.goalPos = FieldConstants.RED_GOAL_POS;
        }
    }
    private void buildPaths(Pose2d startPose, boolean mirror) {
        Pose parkEnd = new Pose (48, 130, 1D/2D*Math.PI);
        Pose shootPreloadPose = SHOOT_PRELOAD_POSE.toPedro();

        if (mirror) {
            parkEnd.mirror();
            shootPreloadPose.mirror();
        }

        Follower follower = robot.follower;
        shootPreloadPath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(startPose.toPedro(), shootPreloadPose)
                )
                .setLinearHeadingInterpolation(startPose.heading, shootPreloadPose.getHeading())
                .build();

        parkPath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(startPose.toPedro(), parkEnd)
                )
                .setLinearHeadingInterpolation(startPose.heading, parkEnd.getHeading())
                .build();
    }

    private void buildCommands() {
        shootPreloadCommand = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new PrepareShootCommand(robot),
                        new FollowPathCommand(robot.follower, shootPreloadPath, true)
                ),
                new WaitCommand(PRELOAD_PRE_SHOOT_DELAY),
                new ShimmyShoot3Ball(robot),
                new WaitForSpindexerYawCommand(robot.spindexer).withTimeout(500),
                new WaitCommand(500)
        );

        parkCommand = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, parkPath),
                        new GoToRestingStateCommand(robot)
                ));
    }


    public void runOpMode(){
        hardwareMap.dcMotor.get("motorFrontLeft").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardwareMap.dcMotor.get("motorFrontLeft").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL, RobotHardware.HardwareOptions.CAMERA);

        robot.init(hardware, telemetry);
        robot.goalPos = team.getGoalPos();




        buildPaths(team.getStartPosAuto(), Team.RED.equals(team));
        buildCommands();
        robot.follower.setMaxPower(1.0);

        waitForStart();

        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                shootPreloadCommand,
                parkCommand
        ));



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

            robot.telemetry.update();
        }
    }

}
