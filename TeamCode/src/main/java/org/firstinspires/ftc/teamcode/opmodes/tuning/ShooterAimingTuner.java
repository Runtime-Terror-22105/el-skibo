package org.firstinspires.ftc.teamcode.opmodes.tuning;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.command.shooter.SpindexerHoming;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.TransferCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToRestingStateCommand;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.subsystems.ShotType;

//@Config
@TeleOp(name="Shooter Aim Tuner", group="Tuning")
public class ShooterAimingTuner extends LinearOpMode {

    private final RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();

    public static double GOAL_RPM = 0.0;

    public static double DIST_TO_GOAL_X = 81; // in mm
    public static double DIST_TO_GOAL_Y = 26; // in mm

    public static double DIST_TO_GOAL = -1; // in mm
    public static double ROBOT_ANGLE_TO_GOAL = -1; // in degrees

    @Override
    public void runOpMode() {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        robot.init(hardware, telemetry);

        waitForStart();

        CommandScheduler.getInstance().schedule(new ParallelCommandGroup(
                new SpindexerHoming(robot.spindexer),
                new GoToRestingStateCommand(robot)
        ));

        while (opModeIsActive()) {
            // Manually clear the bulk read cache. Deleting this would be catastrophic b/c stale
            // vals would be used.
            for (LynxModule hub : hardware.allHubs) {
                hub.clearBulkCache();
            }

//            Pose2d robotPose;
//            if (DIST_TO_GOAL_X != -1 && DIST_TO_GOAL_Y != -1) {
//                robotPose = new Pose2d(DIST_TO_GOAL_X, DIST_TO_GOAL_Y, Math.atan2(DIST_TO_GOAL_Y, DIST_TO_GOAL_X));
//            } else if (DIST_TO_GOAL != -1) {
//                double angleRad;
//                if (ROBOT_ANGLE_TO_GOAL != -1) {
//                    angleRad = Math.toRadians(ROBOT_ANGLE_TO_GOAL);
//                } else {
//                    // note: you might want to test out whether the internal imu works properly separately using InternalIMUTest
//                    angleRad = hardwareMap.get(IMU.class, "imu").getRobotYawPitchRollAngles().getYaw();
//                }
//                double x = DIST_TO_GOAL * Math.cos(angleRad);
//                double y = DIST_TO_GOAL * Math.sin(angleRad);
//                robotPose = new Pose2d(x, y, angleRad);
//            } else {
//                robotPose = new Pose2d(0, 0, 0);
//            }
            Pose2d robotPose = new Pose2d(DIST_TO_GOAL_X, DIST_TO_GOAL_Y, Math.atan2(DIST_TO_GOAL_Y, DIST_TO_GOAL_X));
            Pose2d goalPos = new Pose2d(0, 0, 0);

            robot.shooter.doMath(robotPose, goalPos, ShotType.Arc, 60.0);
            robot.shooter.manualAim(robot.shooter.goalVelocity, robot.shooter.goalPitch, 0);

            if (gamepad1.rightBumperWasPressed()) {
                CommandScheduler.getInstance().schedule(new TransferCommand(robot));
            }

            //robot.shooter.setSpeed(GOAL_RPM);

            CommandScheduler.getInstance().run();

            hardware.write();


            robot.telemetry.addData("Goal Yaw", robot.shooter.goalYaw);
            robot.telemetry.addData("Goal Velocity in/sec", robot.shooter.goalVelocity);
            robot.telemetry.addData("Goal Velocity rpm", robot.shooter.velToRPM(robot.shooter.goalVelocity));
            robot.telemetry.addData("Goal Pitch", robot.shooter.goalPitch);
            robot.telemetry.addData("Current velocity rpm",robot.shooter.getVelocityRpm());
            robot.telemetry.update();

        }

    }

}
