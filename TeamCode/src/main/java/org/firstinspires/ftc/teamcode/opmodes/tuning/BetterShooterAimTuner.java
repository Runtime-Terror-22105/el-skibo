package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static org.firstinspires.ftc.teamcode.FieldConstants.AUTO_ENDING_DATA_KEY;
import static org.firstinspires.ftc.teamcode.FieldConstants.MOTIF_DATA_KEY;
import static org.firstinspires.ftc.teamcode.FieldConstants.RED_KEY;
import static org.firstinspires.ftc.teamcode.FieldConstants.SPINDEXER_POSITION_KEY;
import static org.firstinspires.ftc.teamcode.FieldConstants.TEAM_COLOR_KEY;
import static org.firstinspires.ftc.teamcode.FieldConstants.TELEOP_ENDING_KEY;
import static org.firstinspires.ftc.teamcode.robot.init.RobotState.HANGING_FINAL;
import static org.firstinspires.ftc.teamcode.robot.init.RobotState.INTAKING;
import static org.firstinspires.ftc.teamcode.robot.init.RobotState.READY_TO_SHOOT;
import static org.firstinspires.ftc.teamcode.robot.init.RobotState.RESTING;
import static org.firstinspires.ftc.teamcode.robot.init.RobotState.SHOOTING;
import static org.firstinspires.ftc.teamcode.robot.init.RobotState.TRANSFER;
import static org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem.DEFAULT_SPEED;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.pedroPathing.FtcDashDrawing;
import org.firstinspires.ftc.teamcode.robot.command.DriveCommand;
import org.firstinspires.ftc.teamcode.robot.command.intake.SetIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.robot.command.shooter.AdjustTurretOffsetCommand;
import org.firstinspires.ftc.teamcode.robot.command.shooter.ShootThreeBallsCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.PrepareShootCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToIntakeStateCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToRestingStateCommand;
import org.firstinspires.ftc.teamcode.robot.hardware.TerrorLight;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;
import org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.ArrayUtil;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.Profiler;
import org.opencv.core.Mat;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

@TeleOp(name = "Better Shooter Aim Tuner", group = "Tuning")
@Config
public class BetterShooterAimTuner extends LinearOpMode {
    private final RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();

    public static boolean debug = true;
    public static boolean telemetryBool = true;
    public static boolean blueSide = true;
    public static double velocity = 450;
    public static double hoodAngle = 0.7;
    public static double turretPos = 0;
    public static boolean autoHood  = true;
    public static double spindexOffset = -0.3;
    public static boolean useIntake = true;
    public static boolean tuneGoalPos = false;
    public static double goalPosOffset = 0;
    public Team color;
    private RobotState lastState;
    private RobotState oldstate;

    public static boolean testingAutoShoot = false;



    public static String OUTPUT_FILE = "shooter_tuning_data.csv";

    @Override
    public void runOpMode() {
        Profiler.init();

        hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL, RobotHardware.HardwareOptions.CAMERA);
//        hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL,Rob);
        robot.init(hardware, this);


        Object motif = blackboard.getOrDefault(MOTIF_DATA_KEY, null);
        Object autoEnd = blackboard.getOrDefault(AUTO_ENDING_DATA_KEY, null);
        Object spindexerPosition = blackboard.getOrDefault(SPINDEXER_POSITION_KEY, null);
        Object teleEnd =  blackboard.getOrDefault(TELEOP_ENDING_KEY, null);
        Log.i("Auto", "Ending position after auto " + ((Pose) blackboard.get(AUTO_ENDING_DATA_KEY)));
    if (blueSide){
        this.color = Team.BLUE;
        this.robot.color = Team.BLUE;
    }
    else{
        this.color = Team.RED;
        this.robot.color = Team.RED;

    }

        if (motif != null) {
            //robot.camera.setGlyph((CameraSubsystem.GLYPH) motif);
            //robot.camera.stopScanningForGlyphs();
            //blackboard.put(MOTIF_DATA_KEY, null);
        } else {
//            robot.camera.startScanningForGlyphs();
        }
        blackboard.put(AUTO_ENDING_DATA_KEY, null);
        blackboard.put(TELEOP_ENDING_KEY, null);
        if (autoEnd != null) {
            robot.follower.setStartingPose((Pose) autoEnd);
        }
        else {
            robot.follower.setStartingPose(color.getStartPosNear().toPedro());
        }
//        if (spindexerPosition != null) {
//            robot.spindexer.setHomedSpindexerOffset(-((double) spindexerPosition));
//            blackboard.put(SPINDEXER_POSITION_KEY, null);
//        }

        while (opModeInInit()) {
            for (LynxModule hub : hardware.allHubs) {
                hub.clearBulkCache();
            }

//            CommandScheduler.getInstance().run();

            robot.lightControl.setIsManualLighting(true);
            if(robot.color.equals(Team.RED))
            {
                robot.lightControl.setManualLightColor(TerrorLight.LightColors.RED);
            }
            else if(robot.color.equals(Team.BLUE))
            {
                robot.lightControl.setManualLightColor(TerrorLight.LightColors.BLUE);
            }

            hardware.write();

            FtcDashDrawing.drawDebug(robot.follower);
//            this.showPoem();
            robot.telemetry.update();
        }

        waitForStart();
        lastState = robot.robotState;
//        robot.camera.stopCamera();

        robot.lightControl.setIsManualLighting(false);
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

        GamepadButton manualSpindexLeft = new GamepadButton(gamepad1ex,GamepadKeys.Button.DPAD_LEFT);
        GamepadButton manualSpindexRight = new GamepadButton(gamepad1ex,GamepadKeys.Button.DPAD_RIGHT);

        Trigger intakeButton = new Trigger(() -> gamepad1ex.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.3);
        Trigger reverseIntakeButton = new Trigger(() -> gamepad1ex.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.3);
        GamepadButton relocalizeButton = new GamepadButton(gamepad1ex, GamepadKeys.Button.X);

        GamepadButton shoot3button = new GamepadButton(gamepad1ex, GamepadKeys.Button.RIGHT_BUMPER);
        GamepadButton transferButton = new GamepadButton(gamepad1ex, GamepadKeys.Button.LEFT_BUMPER);

        GamepadButton resetPinpointButton = new GamepadButton(gamepad1ex, GamepadKeys.Button.BACK);

//        GamepadButton adjustTurretLeft = new GamepadButton(gamepad1ex, GamepadKeys.Button.DPAD_LEFT);
//        GamepadButton adjustTurretRight = new GamepadButton(gamepad1ex, GamepadKeys.Button.DPAD_RIGHT);

//        GamepadButton sortButton = new GamepadButton(gamepad1ex, GamepadKeys.Button.DPAD_UP);
        GamepadButton sortButton = new GamepadButton(gamepad1ex, GamepadKeys.Button.B);

//        GamepadButton hangButton = new GamepadButton(gamepad1ex, GamepadKeys.Button.Y);
        GamepadButton hangManualUpButton = new GamepadButton(gamepad1ex, GamepadKeys.Button.DPAD_DOWN);
        GamepadButton hangManualDownButton = new GamepadButton(gamepad1ex, GamepadKeys.Button.DPAD_UP);

        GamepadButton headingLockButton = new GamepadButton(gamepad1ex, GamepadKeys.Button.Y);
        GamepadButton slowSpeedButton = new GamepadButton(gamepad1ex, GamepadKeys.Button.A);

        Trigger threeBallsAreInside = new Trigger(() -> !ArrayUtil.contains(robot.spindexer.getBallPositions(), BallColor.NONE));
//        GamepadButton tapeZoneShoot = new GamepadButton(gamepad1ex, GamepadKeys.Button.CIRCLE);
//        tapeZoneShoot.whenPressed(() -> robot.setShootInTapeZone(!robot.getShootInTapeZone()));
//
//        Trigger botInTapeZone = new Trigger(()-> robot.isInTapeZone() && robot.getShootInTapeZone() && threeBallsAreInside.get());
//
//        botInTapeZone.whenActive(()->new ConditionalCommand(
//                new ConditionalCommand( // if we already did the transfer, just shoot immediately
//                        new ShootThreeBallsCommand(robot),
//                        new SequentialCommandGroup(
//                                new PrepareShootCommand(robot, true),
//                                new ShootThreeBallsCommand(robot)
//                        ),
//                        () -> robot.robotState == READY_TO_SHOOT
//                ),
//                new InstantCommand(() -> {} ),
//                () -> robot.robotState != SHOOTING && robot.robotState != TRANSFER
//        ));

        // todo: implement hang later
//        hangButton.whenPressed(new ChangeHangStateCommand(robot));

        intakeButton.whenActive(new ConditionalCommand(
                new GoToIntakeStateCommand(robot),
                new InstantCommand(() -> {} ),
                () ->  robot.robotState != SHOOTING && robot.robotState != READY_TO_SHOOT && robot.robotState != TRANSFER
        ));
        // todo: test the following
        // this allows that if we press intaking while shooting still   , we'll go to intaking the moment we go to resting
        intakeButton.whileActiveContinuous(new ConditionalCommand(
                new GoToIntakeStateCommand(robot),
                new InstantCommand(() -> {} ),
                () ->  robot.robotState == RESTING && lastState != INTAKING // if we are already intaking and 3 balls inside for resting, we don't want to immediately go back to intaking
        ));
        intakeButton.whenInactive(new ConditionalCommand( // if not full state, we will go to resting
                new GoToRestingStateCommand(robot),
                new InstantCommand(() -> {} ),
                () -> robot.robotState != SHOOTING && robot.robotState != READY_TO_SHOOT && robot.robotState != TRANSFER && robot.robotState != HANGING_FINAL
        ));

        reverseIntakeButton.whenActive(new ConditionalCommand(
                new SetIntakeSpeedCommand(robot.intake, IntakeSubsystem.REVERSE_SPEED),
                new InstantCommand(() -> {} ),
                () -> robot.robotState != SHOOTING //robot.robotState != FULL
        ));
        reverseIntakeButton.whenInactive(new SetIntakeSpeedCommand(robot.intake, 0.0));

        threeBallsAreInside.whenActive(new ConditionalCommand(
                new SequentialCommandGroup(
                        new SetIntakeSpeedCommand(robot.intake, 0),
                        new PrepareShootCommand(robot, true)
                ),
                new InstantCommand(() -> {} ),
                () -> robot.robotState == RESTING || robot.robotState == INTAKING
        ));

        manualSpindexLeft.whileHeld(new AdjustTurretOffsetCommand(robot, true));

        manualSpindexRight.whileHeld(new AdjustTurretOffsetCommand(robot, false));

        slowSpeedButton.whenPressed(() -> robot.drive.toggleSlowSpeed());
        headingLockButton.whenPressed(() -> robot.drive.toggleHeadingLock());

        shoot3button.whenPressed(new ConditionalCommand(
                new ConditionalCommand( // if we already did the transfer, just shoot immediately
                        new ShootThreeBallsCommand(robot,ShootThreeBallsCommand.SPINDEX_TRANSFER_POWER,true),
                        new SequentialCommandGroup(
                                new PrepareShootCommand(robot, true),
                                new ShootThreeBallsCommand(robot,ShootThreeBallsCommand.SPINDEX_TRANSFER_POWER,true)
                        ),
                        () -> robot.robotState == READY_TO_SHOOT
                ),
                new InstantCommand(() -> {} ),
                () -> robot.robotState != SHOOTING && robot.robotState != TRANSFER
        ));

        transferButton.whenPressed(new ConditionalCommand(
                new PrepareShootCommand(robot, true),
                new InstantCommand(() -> {} ),
                () -> robot.robotState != SHOOTING && robot.robotState != TRANSFER
        ));

        relocalizeButton.whenPressed(new InstantCommand(()->{
            robot.robotState = RobotState.SCANNING;
        }));

        resetPinpointButton.whenPressed(new InstantCommand(() -> robot.follower.setStartingPose(robot.follower.getPose())));

//        double hangPos = 0.0;
//        hangManualUpButton.whenPressed(() -> {
//            hardware.hangLeft.setPosition(-1.0);
//            hardware.hangRight.setPosition(1.0);
//        });
//        hangManualUpButton.whenReleased(() -> {
//            hardware.hangLeft.setPosition(0);
//            hardware.hangRight.setPosition(0);
//        });
//        hangManualDownButton.whenPressed(() -> {
//            hardware.hangLeft.setPosition(1.0);
//            hardware.hangRight.setPosition(-1.0);
//        });
//        hangManualDownButton.whenReleased(() -> {
//            hardware.hangLeft.setPosition(0);
//            hardware.hangRight.setPosition(0);
//        });
        //adjustSpindexerLeft.whileHeld(new AdjustSpindexZeroCommand(robot, false));
        //adjustSpindexerRight.whileHeld(new AdjustSpindexZeroCommand(robot, true));

        sortButton.whenPressed(robot::toggleAutoSort);

        // driver 2
        GamepadButton motifPGPButton = new GamepadButton(gamepad2ex, GamepadKeys.Button.X);
        GamepadButton motifGPPButton = new GamepadButton(gamepad2ex, GamepadKeys.Button.Y);
        GamepadButton motifPPGButton = new GamepadButton(gamepad2ex, GamepadKeys.Button.B);


//        motifPGPButton.whenPressed(new InstantCommand(() -> robot.camera.gameGlyph= CameraSubsystem.GLYPH.PGP ));
//        motifGPPButton.whenPressed(new InstantCommand(() -> robot.camera.gameGlyph= CameraSubsystem.GLYPH.GPP ));
//        motifPPGButton.whenPressed(new InstantCommand(() -> robot.camera.gameGlyph= CameraSubsystem.GLYPH.PPG ));

        //homing command executing here

//        SpindexerHoming homing = new SpindexerHoming(robot.spindexer);
        CommandScheduler.getInstance().schedule(new ParallelCommandGroup(
//                new SpindexerHoming(robot.spindexer),
                        new GoToRestingStateCommand(robot),
                        new InstantCommand(() -> robot.shooter.setSpeed(3500D)))
        );


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
                if (!tuneGoalPos) {
                    robot.shooter.manualAim(velocity, hoodAngle, turretPos);
                }
                else{
                    Pose2d goalPos = robot.goalPos.copy();
                    if(goalPosOffset > 0) {
                        if (robot.color == Team.BLUE) {
                            goalPos.x += Math.abs(goalPosOffset);
                        } else {
                            goalPos.x -= Math.abs(goalPosOffset);
                        }
                    }
                    else if (goalPosOffset < 0){
                        goalPos.y += goalPosOffset;
                    }
                    robot.shooter.manualAimGoalPos(velocity, hoodAngle, goalPos);
                }

            } else {
                robot.shooter.isAutoAimOn = true;
            }

            robot.spindexer.enableRamp();
            robot.spindexer.setPidEnabled(false);
            if (useIntake) robot.intake.setSpeed(DEFAULT_SPEED);

            Vector goalToRobot;
            Pose robotPose = robot.follower.getPose();

            if (Team.RED.equals(robot.color)) {
                robotPose = robotPose.mirror();
            }

            goalToRobot = new Vector(new Pose(robotPose.getX(), robotPose.getY()-144D));
            double angle = Math.abs(Angle.angleWrap(goalToRobot.getTheta()));
            if (angle > ((1D/2D)*Math.PI)){
                angle = (1D/4D)*Math.PI;
            }
            if (telemetryBool) robot.telemetry.addData("angle (rad): ", angle);
            if (telemetryBool) robot.telemetry.addData("angle (deg): ", angle *180D/Math.PI);


            if (robot.robotState == INTAKING && oldstate != INTAKING) {
                double dist = Math.sqrt(Math.pow(robot.follower.getPose().getX()-robot.shooter.goalPosLookupTable.get().x, 2) +
                        Math.pow(robot.follower.getPose().getY()-robot.shooter.goalPosLookupTable.get().y, 2));
                double vel = robot.shooter.getGoalVelocity()/6.469;
                String str = "distance: " + dist + " velocity: " + vel + " hood angle: " + hoodAngle;
                if (debug) Log.i("BetterShooterAimTuner", str);



                if (debug) Log.i("BetterShooterAimTuner", "angle" + angle + "offset "+ goalPosOffset);

                // save the data point to a file
                File outputFile = new File(OUTPUT_FILE);
                if (outputFile.exists()) {
                    outputFile.delete(); // delete the file if it already exists
                }
                try (FileOutputStream out = new FileOutputStream(outputFile)) {
                    out.write(str.getBytes());
                    if (debug) Log.i("BetterShooterAimTuner", "Data point saved to " + outputFile.getAbsolutePath());
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
            oldstate = robot.robotState;


            CommandScheduler.getInstance().run();


            hardware.write();

            FtcDashDrawing.drawDot(robot.shooter.goalPosLookupTable.get().toPedro(), "#000000");
            FtcDashDrawing.drawDebug(robot.follower);
            if (telemetryBool) {
                robot.telemetry.addData("Robot Position", robot.follower.getPose());
                robot.telemetry.addData("Goal Position", robot.shooter.goalPosLookupTable.get());
                robot.telemetry.addData("Distance", Math.sqrt(Math.pow(robot.follower.getPose().getX() - robot.shooter.goalPosLookupTable.get().x, 2) + Math.pow(robot.follower.getPose().getY() - robot.shooter.goalPosLookupTable.get().y, 2)));

                robot.telemetry.addData("Goal Yaw", robot.shooter.goalTurretAngle);
                robot.telemetry.addData("Goal Turret Pos w/out offset", ShooterSubsystem.turretAngleToServoPos(robot.shooter.goalTurretAngle));
                robot.telemetry.addData("Goal Turret Pos w/ offset", ShooterSubsystem.turretAngleToServoPos(robot.shooter.goalTurretAngle) + robot.shooter.turretOffset);

                robot.telemetry.addData("Goal Velocity in/sec", robot.shooter.getGoalVelocity());
                robot.telemetry.addData("Goal Velocity rpm", robot.shooter.velToRPM(robot.shooter.getGoalVelocity()));
                robot.telemetry.addData("Current velocity rpm", robot.shooter.getVelocityRpm());
                robot.telemetry.addData("Current velocity in/sec", robot.shooter.getVelocityRpm() / 6.469);

                robot.telemetry.addData("Goal Pitch", robot.shooter.goalPitch);
                robot.telemetry.addData("Goal Hood Pos", robot.shooter.goalPitchPos);
            }

            robot.telemetry.update();
        }

        blackboard.put(AUTO_ENDING_DATA_KEY, null);

        robot.close();

    }

}




