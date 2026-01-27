package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static org.firstinspires.ftc.teamcode.FieldConstants.AUTO_ENDING_DATA_KEY;
import static org.firstinspires.ftc.teamcode.FieldConstants.MOTIF_DATA_KEY;
import static org.firstinspires.ftc.teamcode.FieldConstants.SPINDEXER_POSITION_KEY;
import static org.firstinspires.ftc.teamcode.robot.init.RobotState.INTAKING;
import static org.firstinspires.ftc.teamcode.robot.init.RobotState.READY_TO_SHOOT;
import static org.firstinspires.ftc.teamcode.robot.init.RobotState.RESTING;
import static org.firstinspires.ftc.teamcode.robot.init.RobotState.SHOOTING;
import static org.firstinspires.ftc.teamcode.robot.init.RobotState.TRANSFER;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.pedroPathing.FtcDashDrawing;
import org.firstinspires.ftc.teamcode.robot.command.DriveCommand;
import org.firstinspires.ftc.teamcode.robot.command.intake.SetIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.robot.command.shooter.*;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
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

import org.firstinspires.ftc.teamcode.robot.command.spindexer.AdjustSpindexZeroCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.ChangeSpindexerYawCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.PrepareShootCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToClimbStateCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToIntakeStateCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToRestingStateCommand;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.subsystems.HangSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.vision.CameraSubsystem;
import org.firstinspires.ftc.teamcode.util.ArrayUtil;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.Profiler;

import java.util.List;

@Config
@TeleOp(name="ShooterAimTuner", group="Tuning")
public abstract class ShooterAimingTuner extends LinearOpMode {
    public static boolean LOG_MOTOR_CURRENT = false;

    private final RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();

    public Team color;
    public static ShootingMethod shootingMethod = ShootingMethod.SHOOT_3_BALLS;

    public static boolean debug = true;
    public static boolean telemetryBool = true;
    public static boolean blueSide = true;
    public static double velocity = 450;
    public static double hoodAngle = 0.7;
    public static double turretPos = 0;
    public static boolean autoHood  = true;
    public static boolean tuneGoalPos = false;
    public static double goalPosOffset = 0;
    public static boolean testingAutoShoot = false;

    private long lastLoop = System.nanoTime();

    public enum ShootingMethod{
        SHOOT_3_BALLS,
        SHIMMY_SHOOT,
        PAUSE_SHOOT
    }


    public void setTeam(Team color) {
        if (color == Team.BLUE){
            robot.goalPos = FieldConstants.BLUE_GOAL_POS;
        }
        else {
            robot.goalPos = FieldConstants.RED_GOAL_POS;
        }
        robot.color = color;
    }
    public ShooterAimingTuner(){
        if (blueSide){
            this.setTeam(Team.BLUE);
        }
        else{
            this.setTeam(Team.RED);
        }

    }

    public void runOpMode() {
        Profiler.init();

        hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL, RobotHardware.HardwareOptions.CAMERA);
        robot.init(hardware, telemetry);

        robot.follower.setStartingPose(color.getStartPosNear().toPedro());


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

        GamepadButton manualSpindexLeft = new GamepadButton(gamepad1ex,GamepadKeys.Button.DPAD_LEFT);
        GamepadButton manualSpindexRight = new GamepadButton(gamepad1ex,GamepadKeys.Button.DPAD_RIGHT);

        GamepadButton hangButton = new GamepadButton(gamepad1ex, GamepadKeys.Button.Y);
        Trigger intakeButton = new Trigger(() -> gamepad1ex.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.3);
        Trigger reverseIntakeButton = new Trigger(() -> gamepad1ex.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.3);
        GamepadButton rejectButton = new GamepadButton(gamepad1ex, GamepadKeys.Button.A);
        GamepadButton restingButton = new GamepadButton(gamepad1ex, GamepadKeys.Button.X);

        GamepadButton shoot3button = new GamepadButton(gamepad1ex, GamepadKeys.Button.RIGHT_BUMPER);
        GamepadButton shoot1button = new GamepadButton(gamepad1ex, GamepadKeys.Button.LEFT_BUMPER);

        GamepadButton resetPinpointButton = new GamepadButton(gamepad1ex, GamepadKeys.Button.BACK);

        //GamepadButton adjustSpindexerLeft = new GamepadButton(gamepad1ex, GamepadKeys.Button.DPAD_LEFT);
        //GamepadButton adjustSpindexerRight = new GamepadButton(gamepad1ex, GamepadKeys.Button.DPAD_RIGHT);

        GamepadButton sortButton = new GamepadButton(gamepad1ex, GamepadKeys.Button.DPAD_UP);

        Trigger threeBallsAreInside = new Trigger(() -> !ArrayUtil.contains(robot.spindexer.getBallPositions(), BallColor.NONE));

        Trigger botInTapeZone = new Trigger(()-> robot.isInTapeZone() && robot.getShootInTapeZone());

//        botInTapeZone.whenActive(new InstantCommand()->);

        hangButton.whenPressed(new ConditionalCommand(
                new GoToClimbStateCommand(robot, HangSubsystem.Position.RESTING),
                new GoToClimbStateCommand(robot, HangSubsystem.Position.FULL_90),
                () -> robot.hang.isPtoEngaged()
        ));
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
                () -> robot.robotState != SHOOTING && robot.robotState != READY_TO_SHOOT && robot.robotState != TRANSFER
        ));

        reverseIntakeButton.whenActive(new ConditionalCommand(
                new SetIntakeSpeedCommand(robot.intake, IntakeSubsystem.REVERSE_SPEED),
                new InstantCommand(() -> {} ),
                () -> robot.robotState != SHOOTING //robot.robotState != FULL
        ));
        reverseIntakeButton.whenInactive(new SetIntakeSpeedCommand(robot.intake, 0.0));

        threeBallsAreInside.whenActive(new ConditionalCommand(
                new PrepareShootCommand(robot),
                new InstantCommand(() -> {} ),
                () -> robot.robotState == RESTING || robot.robotState == INTAKING
        ));

        manualSpindexLeft.whileHeld(new ChangeSpindexerYawCommand(robot.spindexer,Math.toRadians(-SpindexerSubsystem.MANUAL_SPINDEXER_DEGREE_CHANGE)));

        manualSpindexRight.whileHeld(new ChangeSpindexerYawCommand(robot.spindexer,Math.toRadians(SpindexerSubsystem.MANUAL_SPINDEXER_DEGREE_CHANGE)));

//        manualSpindexLeft.whenReleased(()->{robot.spindexer.setSpindexerPower(0);});
//
//        manualSpindexRight.whenReleased(()->{robot.spindexer.setSpindexerPower(0);});

        if (shootingMethod == ShootingMethod.SHOOT_3_BALLS){
            shoot3button.whenPressed(new ConditionalCommand(
                    new ConditionalCommand( // if we already did the transfer, just shoot immediately
                            new ShootThreeBallsCommand(robot),
                            new SequentialCommandGroup(
                                    new PrepareShootCommand(robot),
                                    new ShootThreeBallsCommand(robot)
                            ),
                            () -> robot.robotState == READY_TO_SHOOT
                    ),
                    new InstantCommand(() -> {} ),
                    () -> robot.robotState != SHOOTING && robot.robotState != TRANSFER
            ));
        }

        shoot1button.whenPressed(new ConditionalCommand(
                new PrepareShootCommand(robot),
                new InstantCommand(() -> {} ),
                () -> robot.robotState != SHOOTING
        ));

        rejectButton.whenPressed(new ConditionalCommand(
                new StartShooterRejectCommand(robot.shooter),
                new InstantCommand(() -> {} ),
                () -> robot.robotState != SHOOTING //robot.robotState == FULL
        ));

        restingButton.whenPressed(new GoToRestingStateCommand(robot));

        resetPinpointButton.whenPressed(new InstantCommand(() -> robot.follower.setStartingPose(robot.follower.getPose())));

        //adjustSpindexerLeft.whileHeld(new AdjustSpindexZeroCommand(robot, false));
        //adjustSpindexerRight.whileHeld(new AdjustSpindexZeroCommand(robot, true));

        sortButton.whenPressed(robot::toggleAutoSort);

        // driver 2
        GamepadButton motifPGPButton = new GamepadButton(gamepad2ex, GamepadKeys.Button.X);
        GamepadButton motifGPPButton = new GamepadButton(gamepad2ex, GamepadKeys.Button.Y);
        GamepadButton motifPPGButton = new GamepadButton(gamepad2ex, GamepadKeys.Button.B);


        motifPGPButton.whenPressed(new InstantCommand(() -> robot.camera.gameGlyph= CameraSubsystem.GLYPH.PGP ));
        motifGPPButton.whenPressed(new InstantCommand(() -> robot.camera.gameGlyph= CameraSubsystem.GLYPH.GPP ));
        motifPPGButton.whenPressed(new InstantCommand(() -> robot.camera.gameGlyph= CameraSubsystem.GLYPH.PPG ));

        //homing command executing here

//        SpindexerHoming homing = new SpindexerHoming(robot.spindexer);
        CommandScheduler.getInstance().schedule(new ParallelCommandGroup(
//                new SpindexerHoming(robot.spindexer),
                        new GoToRestingStateCommand(robot),
                        new InstantCommand(() -> robot.shooter.setSpeed(3500D)))
        );

        lastLoop = System.nanoTime();
        this.robot.shooter.isAutoAimOn = false;

        while (opModeIsActive()) {
            Profiler.start();

            // Manually clear the bulk read cache. Deleting this would be catastrophic b/c stale
            // vals would be used.
            Profiler.push("clear_cache");
            for (LynxModule hub : hardware.allHubs) {
                hub.clearBulkCache();
            }
            Profiler.pop();

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

            Profiler.push("commands");
            CommandScheduler.getInstance().run();
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
            robot.telemetry.update();
            Profiler.pop();

            Profiler.end();
            Profiler.sendFlamegraph(robot.telemetry);
        }

        blackboard.put(AUTO_ENDING_DATA_KEY, null);
    }


}
