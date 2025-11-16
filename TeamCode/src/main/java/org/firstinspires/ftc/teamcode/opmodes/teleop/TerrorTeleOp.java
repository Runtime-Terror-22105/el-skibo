package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.robot.hardware.TerrorGamepad.State.RISING;
import static org.firstinspires.ftc.teamcode.robot.init.RobotState.CLIMBING;
import static org.firstinspires.ftc.teamcode.robot.init.RobotState.FULL;
import static org.firstinspires.ftc.teamcode.robot.init.RobotState.INTAKING;
import static org.firstinspires.ftc.teamcode.robot.init.RobotState.RESTING;
import static org.firstinspires.ftc.teamcode.robot.init.RobotState.SHOOTING;

import org.firstinspires.ftc.teamcode.robot.command.DriveCommand;
import org.firstinspires.ftc.teamcode.robot.command.WaitForIntakeCommand;
import org.firstinspires.ftc.teamcode.robot.command.shooter.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.PerpetualCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.gamepad.ToggleButtonReader;
import com.seattlesolvers.solverslib.gamepad.TriggerReader;

import org.firstinspires.ftc.teamcode.math.Algebra;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Coordinate;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.TransferCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToClimbStateCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToFullStateCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToIntakeStateCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToRestingStateCommand;
import org.firstinspires.ftc.teamcode.robot.hardware.TerrorGamepad;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.init.StateTag;
import org.firstinspires.ftc.teamcode.robot.subsystems.*;
import org.firstinspires.ftc.teamcode.robot.subsystems.vision.CameraSubsystem;

@Config

public abstract class TerrorTeleOp extends LinearOpMode {
    public static double TURRET_OVERRIDE_COOLDOWN = 2.0; // If you use a manual override on the turret, it will take this long before it starts autoaiming again

    private RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();

    private Pose2d goalPos;

    public void setGoalPos(Pose2d goalPos) {this.goalPos = goalPos;}


    public void runOpMode() {

        hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL);

        robot.init(hardware, telemetry);

        waitForStart();
        GamepadEx gamepad1ex = new GamepadEx(gamepad1);
        GamepadEx gamepad2ex = new GamepadEx(gamepad2);


        // driver 1
        robot.drivetrain
                .setDefaultCommand(new DriveCommand(
                        () -> (double) gamepad1.left_stick_x,
                        () -> (double) gamepad1.left_stick_y,
                        () -> (double) gamepad1.right_stick_x, robot)
                       );

        GamepadButton hangButton = new GamepadButton(gamepad1ex, GamepadKeys.Button.Y);
        GamepadButton intakeButton = new GamepadButton(gamepad1ex, GamepadKeys.Button.B);
        GamepadButton rejectButton = new GamepadButton(gamepad1ex, GamepadKeys.Button.A);
        GamepadButton restingButton = new GamepadButton(gamepad1ex, GamepadKeys.Button.X);

        GamepadButton shoot3button = new GamepadButton(gamepad1ex, GamepadKeys.Button.LEFT_BUMPER);
        GamepadButton shoot1button = new GamepadButton(gamepad1ex, GamepadKeys.Button.RIGHT_BUMPER);

//        hangButton.whenPressed(new GoToClimbStateCommand(robot));
        intakeButton.whenPressed(new ConditionalCommand(
                new SequentialCommandGroup(
                    new GoToIntakeStateCommand(robot, new TransferCommand(robot.spindexer)),
                    new WaitForIntakeCommand(robot),
                    new GoToFullStateCommand(robot)
                ),
                new InstantCommand(() -> {} ),
                () -> robot.robotState != FULL && robot.robotState != SHOOTING
        ));
        shoot3button.whenPressed(new ConditionalCommand(
                new ShootThreeBallsCommand(robot.shooter,robot.spindexer),
                new InstantCommand(() -> {} ),
                () -> robot.robotState == FULL
        ));
        shoot1button.whenPressed(new ConditionalCommand(
                new ShootOneBallCommand(robot.shooter),
                new InstantCommand(() -> {} ),
                () -> robot.robotState == FULL
        ));
        rejectButton.whenPressed(new ConditionalCommand(
                new StartShooterRejectCommand(robot.shooter),
                new InstantCommand(() -> {} ),
                () -> robot.robotState == FULL
        ));
        restingButton.whenPressed(new GoToRestingStateCommand(robot));


        // driver 2
        GamepadButton motifPGPButton = new GamepadButton(gamepad2ex, GamepadKeys.Button.X);
        GamepadButton motifGPPButton = new GamepadButton(gamepad2ex, GamepadKeys.Button.Y);
        GamepadButton motifPPGButton = new GamepadButton(gamepad2ex, GamepadKeys.Button.B);


        motifPGPButton.whenPressed(new InstantCommand(() -> robot.camera.gameGlyph= CameraSubsystem.GLYPH.PGP ));
        motifGPPButton.whenPressed(new InstantCommand(() -> robot.camera.gameGlyph= CameraSubsystem.GLYPH.GPP ));
        motifPPGButton.whenPressed(new InstantCommand(() -> robot.camera.gameGlyph= CameraSubsystem.GLYPH.PPG ));

        // homing command executing here
//        SpindexerHoming homingCommand = new SpindexerHoming(robot.spindexer);
//        CommandScheduler.getInstance().schedule(homingCommand);

        while (opModeIsActive()) {
            for (LynxModule hub : hardware.allHubs) {
                hub.clearBulkCache();
            }

//            if(robot.robotState == INTAKING || robot.robotState == SHOOTING || robot.robotState == FULL){
//                robot.shooter.doAutoShoot(this.goalPos);
//            }
//            else {
//                robot.shooter.isAutoAimOn = false;
//            }

            CommandScheduler.getInstance().run();

            robot.telemetry.addData("Ball Positions", robot.spindexer.getBallPositions());
            robot.telemetry.addData("Yaw Goal", robot.shooter.goalYaw);
            robot.telemetry.update();
            hardware.write();
        }

    }


}
