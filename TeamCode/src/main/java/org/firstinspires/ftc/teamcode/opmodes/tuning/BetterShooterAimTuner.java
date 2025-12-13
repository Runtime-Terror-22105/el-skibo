package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static org.firstinspires.ftc.teamcode.FieldConstants.AUTO_ENDING_DATA_KEY;
import static org.firstinspires.ftc.teamcode.FieldConstants.MOTIF_DATA_KEY;
import static org.firstinspires.ftc.teamcode.robot.init.RobotState.INTAKING;
import static org.firstinspires.ftc.teamcode.robot.init.RobotState.READY_TO_SHOOT;
import static org.firstinspires.ftc.teamcode.robot.init.RobotState.RESTING;
import static org.firstinspires.ftc.teamcode.robot.init.RobotState.SHOOTING;

import android.util.Log;

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

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.opmodes.teleop.TerrorTeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.FtcDashDrawing;
import org.firstinspires.ftc.teamcode.robot.command.DriveCommand;
import org.firstinspires.ftc.teamcode.robot.command.intake.SetIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.robot.command.shooter.AdjustTurretOffsetCommand;
import org.firstinspires.ftc.teamcode.robot.command.shooter.ShootThreeBallsCommand;
import org.firstinspires.ftc.teamcode.robot.command.shooter.StartShooterRejectCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.PrepareShootCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToClimbStateCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToIntakeStateCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToRestingStateCommand;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.vision.CameraSubsystem;

@TeleOp(name = "Better Shooter Aim Tuner", group = "Tuning")
@Config
public class BetterShooterAimTuner extends LinearOpMode {
    private final RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();

    public static double velocity = 450;
    public static double pitch = ShooterSubsystem.hoodPosMin;
    public static double turretPos = 0;

    @Override
    public void runOpMode() {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        robot.init(hardware, telemetry);
        robot.shooter.isAutoAimOn = false;
        robot.goalPos = FieldConstants.BLUE_GOAL_POS;
        robot.color = Team.BLUE;
        robot.follower.setStartingPose(robot.color.getStartPosAuto().toPedro());


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

        GamepadButton hangButton = new GamepadButton(gamepad1ex, GamepadKeys.Button.Y);
        Trigger intakeButton = new Trigger(() -> gamepad1ex.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.3);
        Trigger reverseIntakeButton = new Trigger(() -> gamepad1ex.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.3);
        GamepadButton rejectButton = new GamepadButton(gamepad1ex, GamepadKeys.Button.A);
        GamepadButton restingButton = new GamepadButton(gamepad1ex, GamepadKeys.Button.X);

        GamepadButton shoot3button = new GamepadButton(gamepad1ex, GamepadKeys.Button.RIGHT_BUMPER);
        GamepadButton shoot1button = new GamepadButton(gamepad1ex, GamepadKeys.Button.LEFT_BUMPER);

        GamepadButton resetPinpointButton = new GamepadButton(gamepad1ex, GamepadKeys.Button.BACK);

        GamepadButton adjustTurretLeft = new GamepadButton(gamepad1ex, GamepadKeys.Button.DPAD_LEFT);
        GamepadButton adjustTurretRight = new GamepadButton(gamepad1ex, GamepadKeys.Button.DPAD_RIGHT);

        GamepadButton sortButton = new GamepadButton(gamepad1ex, GamepadKeys.Button.DPAD_UP);

        Trigger threeBallsAreInside = new Trigger(() -> {
            final char[] balls = robot.spindexer.getBallPositions();
            return balls[0] != 'N' && balls[1] != 'N' && balls[2] != 'N';
        });

        hangButton.whenPressed(new GoToClimbStateCommand(robot));
        intakeButton.whenActive(new ConditionalCommand(
                new SequentialCommandGroup(
                        new GoToIntakeStateCommand(robot)
//                    new WaitForIntakeCommand(robot),
//                    new GoToFullStateCommand(robot)
                ),
                new InstantCommand(() -> {} ),
                () ->  robot.robotState != SHOOTING && robot.robotState != READY_TO_SHOOT
        ));
        intakeButton.whenInactive(new ConditionalCommand( // if not full state, we will go to resting
                new GoToRestingStateCommand(robot),
                new InstantCommand(() -> {} ),
                () -> robot.robotState != SHOOTING && robot.robotState != READY_TO_SHOOT
        ));

        reverseIntakeButton.whenActive(new ConditionalCommand(
                new SetIntakeSpeedCommand(robot.intake, -1.0),
                new InstantCommand(() -> {} ),
                () -> robot.robotState != SHOOTING //robot.robotState != FULL
        ));
        reverseIntakeButton.whenInactive(new SetIntakeSpeedCommand(robot.intake, 0.0));

        threeBallsAreInside.whenActive(new ConditionalCommand(
                new PrepareShootCommand(robot),
                new InstantCommand(() -> {} ),
                () -> robot.robotState == RESTING || robot.robotState == INTAKING
        ));

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
                () -> robot.robotState != SHOOTING
        ));

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

        adjustTurretLeft.whileHeld(new AdjustTurretOffsetCommand(robot, false));
        adjustTurretRight.whileHeld(new AdjustTurretOffsetCommand(robot, true));

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

            robot.shooter.manualAim(velocity, pitch, turretPos);

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




