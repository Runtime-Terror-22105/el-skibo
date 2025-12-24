package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static org.firstinspires.ftc.teamcode.FieldConstants.AUTO_ENDING_DATA_KEY;
import static org.firstinspires.ftc.teamcode.robot.init.RobotState.INTAKING;
import static org.firstinspires.ftc.teamcode.robot.init.RobotState.READY_TO_SHOOT;
import static org.firstinspires.ftc.teamcode.robot.init.RobotState.SHOOTING;
import static org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem.DEFAULT_SPEED;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.pedroPathing.FtcDashDrawing;
import org.firstinspires.ftc.teamcode.robot.command.DriveCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToIntakeStateCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToRestingStateCommand;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

@TeleOp(name = "Better Shooter Aim Tuner", group = "Tuning")
@Config
public class BetterShooterAimTuner extends LinearOpMode {
    private final RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();

    public static double velocity = 450;
    public static double hoodAngle = 0.7;
    public static double turretPos = 0;
    public static boolean autoHood  = true;
    public static double spindexOffset = -0.3;
    public static boolean useIntake = false;

    public static boolean testingAutoShoot = false;

    public static String OUTPUT_FILE = "shooter_tuning_data.csv";

    @Override
    public void runOpMode() {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        robot.init(hardware, telemetry);
        robot.shooter.isAutoAimOn = false;
        robot.goalPos = FieldConstants.BLUE_GOAL_POS;
        robot.color = Team.BLUE;
        robot.follower.setStartingPose(robot.color.getStartPosNear().toPedro());


        waitForStart();

        GamepadEx gamepad1ex = new GamepadEx(gamepad1);
        GamepadEx gamepad2ex = new GamepadEx(gamepad2);


        // driver 1
        robot.follower.startTeleOpDrive();
        robot.drive
                .setDefaultCommand(new DriveCommand(
                        () -> (double) gamepad1.left_stick_x,
                        () -> (double) gamepad1.left_stick_y,
                        () -> (double) gamepad1.right_stick_x, robot)
                );

        Trigger intakeButton = new Trigger(() -> gamepad1ex.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.3);

        intakeButton.whenActive(new ConditionalCommand(
                new SequentialCommandGroup(
                        new GoToIntakeStateCommand(robot)
//                    new WaitForIntakeCommand(robot),
//                    new GoToFullStateCommand(robot)
                ),
                new InstantCommand(() -> {} ),
                () ->  robot.robotState != SHOOTING && robot.robotState != READY_TO_SHOOT && useIntake
        ));
        intakeButton.whenInactive(new ConditionalCommand( // if not full state, we will go to resting
                new GoToRestingStateCommand(robot),
                new InstantCommand(() -> {} ),
                () -> robot.robotState != SHOOTING && robot.robotState != READY_TO_SHOOT
        ));


        //homing command executing here

//        SpindexerHoming homing = new SpindexerHoming(robot.spindexer);
        CommandScheduler.getInstance().schedule(new ParallelCommandGroup(
//                new SpindexerHoming(robot.spindexer),
                        new GoToRestingStateCommand(robot),
                        new InstantCommand(() -> robot.shooter.setSpeed(3500D)))
        );

        RobotState oldstate = robot.robotState;
        while (opModeIsActive()) {
            // Manually clear the bulk read cache. Deleting this would be catastrophic b/c stale
            // vals would be used.
            for (LynxModule hub : hardware.allHubs) {
                hub.clearBulkCache();
            }

//            char[] balls = robot.spindexer.getBallPositions();
//            if (balls[0] != 'N' && balls[1] != 'N' && balls[2] != 'N') {
//                CommandScheduler.getInstance().schedule(new GoToFullStateCommand(robot));
//            }
            if (!testingAutoShoot) {
                if (autoHood) {
                    robot.shooter.manualAim(velocity, hoodAngle, turretPos);
                } else {
                    robot.shooter.manualAimAutoHood(velocity, turretPos);
                }
            } else {
                robot.shooter.isAutoAimOn = true;
            }

            robot.spindexer.enableRamp();
            robot.spindexer.setPidEnabled(false);
            if (useIntake) robot.intake.setSpeed(DEFAULT_SPEED);


            if (robot.robotState == INTAKING && oldstate != INTAKING) {
                double dist = Math.sqrt(Math.pow(robot.follower.getPose().getX()-robot.shooter.goalPosLookupTable.get().x, 2) +
                        Math.pow(robot.follower.getPose().getY()-robot.shooter.goalPosLookupTable.get().y, 2));
                double vel = robot.shooter.getGoalVelocity()/6.469;
                String str = "distance: " + dist + " velocity: " + vel + " hood angle: " + hoodAngle;
                Log.d("data point", str);

                // save the data point to a file
                File outputFile = new File(OUTPUT_FILE);
                if (outputFile.exists()) {
                    outputFile.delete(); // delete the file if it already exists
                }
                try (FileOutputStream out = new FileOutputStream(outputFile)) {
                    out.write(str.getBytes());
                    Log.d("data point", "Data point saved to " + outputFile.getAbsolutePath());
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
            oldstate = robot.robotState;


            CommandScheduler.getInstance().run();


            hardware.write();

            FtcDashDrawing.drawDebug(robot.follower);
            robot.telemetry.addData("Robot Position", robot.follower.getPose());
            robot.telemetry.addData("Goal Position", robot.shooter.goalPosLookupTable.get());
            robot.telemetry.addData("Distance", Math.sqrt(Math.pow(robot.follower.getPose().getX()-robot.shooter.goalPosLookupTable.get().x, 2) + Math.pow(robot.follower.getPose().getY()-robot.shooter.goalPosLookupTable.get().y, 2)));

            robot.telemetry.addData("Goal Yaw", robot.shooter.goalTurretAngle);
            robot.telemetry.addData("Goal Turret Pos w/out offset", robot.shooter.goalTurretPos);
            robot.telemetry.addData("Goal Turret Pos w/ offset", robot.shooter.goalTurretPos+ robot.shooter.turretOffset);

            robot.telemetry.addData("Goal Velocity in/sec", robot.shooter.getGoalVelocity());
            robot.telemetry.addData("Goal Velocity rpm", robot.shooter.velToRPM(robot.shooter.getGoalVelocity()));
            robot.telemetry.addData("Current velocity rpm", robot.shooter.getVelocityRpm());
            robot.telemetry.addData("Current velocity in/sec", robot.shooter.getVelocityRpm()/6.469);

            robot.telemetry.addData("Goal Pitch", robot.shooter.goalPitch);
            robot.telemetry.addData("Goal Hood Pos", robot.shooter.goalPitchPos);

            robot.telemetry.update();
        }

        blackboard.put(AUTO_ENDING_DATA_KEY, null);


    }

}




