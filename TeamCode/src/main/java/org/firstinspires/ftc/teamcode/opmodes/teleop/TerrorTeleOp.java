package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.robot.hardware.TerrorGamepad.State.RISING;
import static org.firstinspires.ftc.teamcode.robot.init.RobotState.CLIMBING;
import static org.firstinspires.ftc.teamcode.robot.init.RobotState.FULL;
import static org.firstinspires.ftc.teamcode.robot.init.RobotState.INTAKING;
import static org.firstinspires.ftc.teamcode.robot.init.RobotState.RESTING;
import static org.firstinspires.ftc.teamcode.robot.init.RobotState.SHOOTING;

import org.firstinspires.ftc.teamcode.robot.command.DriveCommand;
import org.firstinspires.ftc.teamcode.robot.command.shooter.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.PerpetualCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
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

    public static double ROTATION_MULTIPLIER = 0.56;
    public static double TURRET_OVERRIDE_COOLDOWN = 2.0; // If you use a manual override on the turret, it will take this long before it starts autoaiming again

    private RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();

    private boolean isFieldCentric = false;
    private Pose2d goalPos;


    public void setFieldCentric(boolean fieldCentric) {
        this.isFieldCentric = fieldCentric;
    }
    public void setGoalPos(Pose2d goalPos) {this.goalPos = goalPos;}


    public void runOpMode(){

        hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL);

        robot.init(hardware, telemetry);

        waitForStart();
        GamepadEx gamepad1ex = new GamepadEx(gamepad1);
        GamepadEx gamepad2ex = new GamepadEx(gamepad2);


        // driver 1
        robot.drivetrain
                .setDefaultCommand(new PerpetualCommand(new DriveCommand(
                        () -> (double) gamepad1.left_stick_x,
                        () -> (double) gamepad1.left_stick_y,
                        () -> (double) gamepad1.right_stick_y, robot))
                       );

        GamepadButton hangButton = new GamepadButton(gamepad1ex, GamepadKeys.Button.Y);
        GamepadButton intakeButton = new GamepadButton(gamepad1ex, GamepadKeys.Button.B);
        GamepadButton rejectButton = new GamepadButton(gamepad1ex, GamepadKeys.Button.A);
        GamepadButton restingButton = new GamepadButton(gamepad1ex, GamepadKeys.Button.X);

        GamepadButton shoot3button = new GamepadButton(gamepad1ex, GamepadKeys.Button.LEFT_BUMPER);
        GamepadButton shoot1button = new GamepadButton(gamepad1ex, GamepadKeys.Button.RIGHT_BUMPER);

        hangButton.whenPressed(new GoToClimbStateCommand(robot));
        intakeButton.whenPressed(new GoToIntakeStateCommand(robot, new TransferCommand(robot.spindexer)));
        shoot3button.whenPressed(new ShootThreeBallsCommand(robot.shooter,robot.spindexer));
        shoot1button.whenPressed(new ShootOneBallCommand(robot.shooter));
        rejectButton.whenPressed(new StartShooterRejectCommand(robot.shooter));
        restingButton.whenPressed(new GoToRestingStateCommand(robot));


        // driver 2
        GamepadButton motifPGPButton = new GamepadButton(gamepad2ex, GamepadKeys.Button.X);
        GamepadButton motifGPPButton = new GamepadButton(gamepad2ex, GamepadKeys.Button.Y);
        GamepadButton motifPPGButton = new GamepadButton(gamepad2ex, GamepadKeys.Button.B);


        motifPGPButton.whenPressed(new InstantCommand(() -> robot.camera.gameGlyph= CameraSubsystem.GLYPH.PGP ));
        motifGPPButton.whenPressed(new InstantCommand(() -> robot.camera.gameGlyph= CameraSubsystem.GLYPH.GPP ));
        motifPPGButton.whenPressed(new InstantCommand(() -> robot.camera.gameGlyph= CameraSubsystem.GLYPH.PPG ));

        // homing command executing here
        SpindexerHoming homingCommand = new SpindexerHoming(robot.spindexer);
        CommandScheduler.getInstance().schedule(homingCommand);

        while (opModeIsActive()){
            for (LynxModule hub : hardware.allHubs) {
                hub.clearBulkCache();
            }

            // driving
            double deadzone_amt = 0;
            double left_x = gamepad1ex.getLeftX();
            double left_y = -gamepad1ex.getLeftY();
            double right_x = gamepad1ex.getRightX();
            left_x = Math.signum(left_x) * Algebra.mapRange(Math.abs(left_x), deadzone_amt, 1.0, 0.0, 1.0);
            left_y = Math.signum(left_y) * Algebra.mapRange(Math.abs(left_y), deadzone_amt, 1.0, 0.0, 1.0);
            right_x = Math.signum(right_x) * Algebra.mapRange(Math.abs(right_x), deadzone_amt, 1.0, 0.0, 1.0);
            Coordinate direction = new Coordinate(slr(left_x), slr(left_y));
            double rotation = slr(right_x)*ROTATION_MULTIPLIER;

            if(robot.robotState == INTAKING || robot.robotState == SHOOTING || robot.robotState == FULL){
                robot.shooter.doAutoShoot(this.goalPos);
            }
            else {
                robot.shooter.isAutoAimOn = false;
            }

            this.robot.drivetrain.move(direction, rotation);


            CommandScheduler.getInstance().run();

            robot.telemetry.addData("Ball Positions", robot.spindexer.getBallPositions());
            robot.telemetry.addData("Yaw Goal", robot.shooter.goalYaw);
        }

    }



    public double slr(double joystick_value) {
        return Math.pow(joystick_value, 5);
    }



}
