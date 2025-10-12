package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.robot.hardware.TerrorGamepad.State.RISING;
import static org.firstinspires.ftc.teamcode.robot.init.RobotState.CLIMBING;
import static org.firstinspires.ftc.teamcode.robot.init.RobotState.INTAKING;
import static org.firstinspires.ftc.teamcode.robot.init.RobotState.RESTING;
import static org.firstinspires.ftc.teamcode.robot.init.RobotState.SHOOTING;
import org.firstinspires.ftc.teamcode.robot.command.shooter.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.gamepad.ToggleButtonReader;
import com.seattlesolvers.solverslib.gamepad.TriggerReader;

import org.firstinspires.ftc.teamcode.math.Algebra;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Coordinate;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToClimbStateCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToIntakeStateCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToRestingStateCommand;
import org.firstinspires.ftc.teamcode.robot.hardware.TerrorGamepad;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.init.StateTag;
import org.firstinspires.ftc.teamcode.robot.subsystems.ShooterSubsystem;

@Config
public abstract class TerrorTeleOp extends LinearOpMode {

    public static double ROTATION_MULTIPLIER = 0.56; // If you use a manual override on the turret, it will take this long before it starts autoaiming again
    public static double TURRET_OVERRIDE_COOLDOWN = 2.0;

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
        GamepadEx gamepad1ex=new GamepadEx(gamepad1);



        GamepadButton hangButton = new GamepadButton(gamepad1ex, GamepadKeys.Button.Y);
        GamepadButton intakeButton = new GamepadButton(gamepad1ex, GamepadKeys.Button.B);
        GamepadButton rejectButton = new GamepadButton(gamepad1ex, GamepadKeys.Button.A);
        GamepadButton restingButton = new GamepadButton(gamepad1ex, GamepadKeys.Button.X);

        GamepadButton shoot3button = new GamepadButton(gamepad1ex, GamepadKeys.Button.LEFT_BUMPER);
        GamepadButton shoot1button = new GamepadButton(gamepad1ex, GamepadKeys.Button.RIGHT_BUMPER);


        hangButton.whenPressed(new GoToClimbStateCommand(robot));
        intakeButton.whenPressed(new GoToIntakeStateCommand(robot));
        shoot3button.whenPressed(new ShootThreeBallsCommand(robot.shooter));
        shoot1button.whenPressed(new ShootOneBallCommand(robot.shooter));
        rejectButton.whenPressed(new StartShooterRejectCommand(robot.shooter));
        restingButton.whenPressed(new GoToRestingStateCommand(robot));








        while (opModeIsActive()){
            for (LynxModule hub : hardware.allHubs) {
                hub.clearBulkCache();
            }

            //gamepad 1

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

        }

    }



    public double slr(double joystick_value) {
        return Math.pow(joystick_value, 5);
    }



}
