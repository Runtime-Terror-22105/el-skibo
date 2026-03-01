package org.firstinspires.ftc.teamcode.opmodes.teleop;

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

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.pedroPathing.FtcDashDrawing;
import org.firstinspires.ftc.teamcode.robot.command.DriveCommand;
import org.firstinspires.ftc.teamcode.robot.command.intake.SetIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.robot.command.shooter.AdjustTurretOffsetCommand;
import org.firstinspires.ftc.teamcode.robot.command.shooter.ShootThreeBallsCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.PrepareShootCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToIntakeStateCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToRestingStateCommand;
import org.firstinspires.ftc.teamcode.robot.hardware.TerrorLight;
import org.firstinspires.ftc.teamcode.robot.hardware.motors.TerrorSwyftCRServo;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;
import org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.ArrayUtil;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.Profiler;

@Config
public abstract class TerrorTeleOp extends LinearOpMode {
    // todo: delete this once hang is tested
    public static double MANUAL_HANG_SPEED = 0.5;

    public static boolean LOG_MOTOR_CURRENT = false;

    private final RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();

    public Team color;

    public boolean SAVE_LOCATION_TELEOP = false;


    private long lastLoop = System.nanoTime();

    public void setTeam(Team color) {
        if (color == Team.BLUE){
            robot.goalPos = FieldConstants.BLUE_GOAL_POS;
        }
        else {
            robot.goalPos = FieldConstants.RED_GOAL_POS;
        }
        robot.color = color;
    }
    public TerrorTeleOp(Team color){
        this.color = color;


    }

    public TerrorTeleOp() {

        SAVE_LOCATION_TELEOP = true;
    }

    public void runOpMode() {
        Profiler.init();

        hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL, RobotHardware.HardwareOptions.CAMERA);
//        hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL,Rob);
        robot.init(hardware, this);

        this.setTeam(color);
        Object motif = blackboard.getOrDefault(MOTIF_DATA_KEY, null);
        Object autoEnd = blackboard.getOrDefault(AUTO_ENDING_DATA_KEY, null);
        Object spindexerPosition = blackboard.getOrDefault(SPINDEXER_POSITION_KEY, null);
        Object teleEnd =  blackboard.getOrDefault(TELEOP_ENDING_KEY, null);
        Log.i("Auto", "Ending position after auto " + ((Pose) blackboard.get(AUTO_ENDING_DATA_KEY)));

        if (SAVE_LOCATION_TELEOP){
            if (RED_KEY == blackboard.getOrDefault(TEAM_COLOR_KEY, null)){
                this.setTeam(Team.RED);
            }
            else{
                this.setTeam(Team.BLUE);
            }
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
        } else if (teleEnd != null && SAVE_LOCATION_TELEOP){
            robot.follower.setStartingPose((Pose) teleEnd);
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

            CommandScheduler.getInstance().run();

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
        GamepadButton hangManualUpButton = new GamepadButton(gamepad1ex, GamepadKeys.Button.DPAD_UP);
        GamepadButton hangManualDownButton = new GamepadButton(gamepad1ex, GamepadKeys.Button.DPAD_DOWN);

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
        // this allows that if we press intaking while shooting still, we'll go to intaking the moment we go to resting
        intakeButton.whileActiveContinuous(new ConditionalCommand(
                new GoToIntakeStateCommand(robot),
                new InstantCommand(() -> {} ),
                () ->  robot.robotState != INTAKING && robot.robotState != SHOOTING && robot.robotState != READY_TO_SHOOT && robot.robotState != TRANSFER
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
                new PrepareShootCommand(robot, true),
                new InstantCommand(() -> {} ),
                () -> robot.robotState == RESTING || robot.robotState == INTAKING
        ));

        manualSpindexLeft.whileHeld(new AdjustTurretOffsetCommand(robot, true));

        manualSpindexRight.whileHeld(new AdjustTurretOffsetCommand(robot, false));

        slowSpeedButton.whenPressed(() -> robot.drive.toggleSlowSpeed());
        headingLockButton.whenPressed(() -> robot.drive.toggleHeadingLock());

        shoot3button.whenPressed(new ConditionalCommand(
                new ConditionalCommand( // if we already did the transfer, just shoot immediately
                        new ShootThreeBallsCommand(robot),
                        new SequentialCommandGroup(
                                new PrepareShootCommand(robot, true),
                                new ShootThreeBallsCommand(robot)
                        ),
                        () -> robot.robotState == READY_TO_SHOOT
                ),
                new InstantCommand(() -> {} ),
                () -> robot.robotState != SHOOTING && robot.robotState != TRANSFER
        ));

        transferButton.whenPressed(new ConditionalCommand(
                new PrepareShootCommand(robot),
                new InstantCommand(() -> {} ),
                () -> robot.robotState != SHOOTING && robot.robotState != TRANSFER
        ));

        relocalizeButton.whenPressed(new InstantCommand(()->{
            robot.camera.scheduleRelocalizeRequest();
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

//            char[] balls = robot.spindexer.getBallPositions();
//            if (balls[0] != 'N' && balls[1] != 'N' && balls[2] != 'N') {
//                CommandScheduler.getInstance().schedule(new GoToFullStateCommand(robot));
//            }

            Profiler.push("commands");
            CommandScheduler.getInstance().run();

            if (hangManualUpButton.get()) {
                robot.robotState = RobotState.HANGING_FINAL;
                hardware.hangLeft.setPower(TerrorSwyftCRServo.Power.FORWARD);
                hardware.hangRight.setPower(TerrorSwyftCRServo.Power.FORWARD);
            } else if (hangManualDownButton.get()) {
                robot.robotState = RobotState.HANGING_FINAL;
                hardware.hangLeft.setPower(TerrorSwyftCRServo.Power.HOME);
                hardware.hangRight.setPower(TerrorSwyftCRServo.Power.HOME);
            } else if (robot.getState() != HANGING_FINAL) {
                hardware.hangLeft.setPwmEnable(false);
                hardware.hangRight.setPwmEnable(false);
            }
            Profiler.pop();

            Profiler.push("hwWrite");
            hardware.write();
            Profiler.pop();

            Profiler.push("debug");
            FtcDashDrawing.drawDebug(robot.follower);
            long time = System.nanoTime();
            long dt = time - lastLoop;
            lastLoop = time;

            if (LOG_MOTOR_CURRENT) {
                double flc = hardware.motorFrontLeft.getCurrent(CurrentUnit.AMPS);
                double frc = hardware.motorFrontRight.getCurrent(CurrentUnit.AMPS);
                double blc = hardware.motorRearLeft.getCurrent(CurrentUnit.AMPS);
                double brc = hardware.motorRearRight.getCurrent(CurrentUnit.AMPS);
                double sl = hardware.shooterLeft.getCurrent(CurrentUnit.AMPS);
                double sr = hardware.shooterRight.getCurrent(CurrentUnit.AMPS);
                double spr = hardware.spindexerRotate.getCurrent(CurrentUnit.AMPS);
                double it = hardware.intake.getCurrent(CurrentUnit.AMPS);


                double totalCurrent = flc + frc + blc + brc + sl + sr + spr + it;

                robot.telemetry.addData("Front Left DT (A)", String.format("%.2f", flc));
                robot.telemetry.addData("Front Right DT (A)", String.format("%.2f", frc));
                robot.telemetry.addData("Rear Left DT (A)", String.format("%.2f", blc));
                robot.telemetry.addData("Rear Right DT (A)", String.format("%.2f", brc));
                robot.telemetry.addData("Shooter Left (A)", String.format("%.2f", sl));
                robot.telemetry.addData("Shooter Right (A)", String.format("%.2f", sr));
                robot.telemetry.addData("Spindexer (A)", String.format("%.2f", spr));
                robot.telemetry.addData("Intake (A)", String.format("%.2f", it));
                robot.telemetry.addData("Total Current (A)", String.format("%.2f", totalCurrent));
            }
            double dist = Math.sqrt(Math.pow(robot.follower.getPose().getX()-robot.goalPos.x, 2) +
                    Math.pow(robot.follower.getPose().getY()-robot.goalPos.y, 2));
            robot.telemetry.addData("Distance (in)", dist);
            robot.telemetry.addData("Loop Time (ms)", String.format("%.2f", dt / 1e6));
            robot.telemetry.addData("Robot position", robot.follower.getPose().toString());
            robot.telemetry.addData("Goal Velocity", robot.shooter.getGoalVelocity());
            robot.telemetry.addData("Current velocity", robot.shooter.getVelocityRpm());

            robot.telemetry.update();
            Profiler.pop();

            Profiler.end();
            Profiler.sendFlamegraph(robot.telemetry);
        }
        if (SAVE_LOCATION_TELEOP){
            blackboard.put(TELEOP_ENDING_KEY, robot.follower.getPose());
        }
        blackboard.put(AUTO_ENDING_DATA_KEY, null);
    }


}
