package org.firstinspires.ftc.teamcode.robot.command;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.MathFunctions;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.math.Algebra;
import org.firstinspires.ftc.teamcode.math.controllers.PidfController;
import org.firstinspires.ftc.teamcode.robot.init.Robot;

import java.util.function.Supplier;

@Config
public class DriveCommand extends CommandBase {
    public static double HEADING_LOCK_ANGLE = Math.toRadians(180);
    private PidfController headingController;

    public static double SLOW_SPEED_FORWARD = 0.2;
    public static double SLOW_SPEED_STRAFE = 0.35;
    public static double SLOW_SPEED_ROTATION = 0.5;

    public static double ROTATION_MULTIPLIER_14V = 1.0;
    public static double ROTATION_MULTIPLIER_12V = 1.0;

    private final Robot robot;
    private final Supplier<Double> x, y, turn;

    public DriveCommand(
            Supplier<Double> x,
            Supplier<Double> y,
            Supplier<Double> turn, Robot robot) {
        this.robot = robot;
        this.x = x;
        this.y = y;
        this.turn = turn;
        this.headingController = new PidfController(robot.follower.constants.coefficientsHeadingPIDF);

        super.addRequirements(robot.drive);
    }

    @Override
    public void initialize() {
//        robot.follower.startTeleOpDrive();
    }

    @Override
    public void execute() {
        //add more stuff from before i dont want to do this
        double rotationMultiplier = Algebra.mapRangeNoClamp(robot.hardware.initialVoltage, 12, 14, ROTATION_MULTIPLIER_12V, ROTATION_MULTIPLIER_14V);
        double left_y = -y.get();
        double left_x = -x.get();
        double right_x = -turn.get();;

//        Log.d("DriveCommand", Boolean.toString(robot.drive.slowSpeed));
        if (robot.drive.slowSpeed) {
            left_y *= SLOW_SPEED_FORWARD;
            left_x *= SLOW_SPEED_STRAFE;
            right_x *= SLOW_SPEED_ROTATION;
        }

        double rotation = right_x * rotationMultiplier;
        if (robot.drive.isHeadingLocked() ) {
            rotation = headingController.calculatePower(getHeadingError(), 0);
        }
        if(robot.getState().isHang())
        {
            return;
        }
//        else if(!(robot.drive.isHoldPosition() && robot.drive.usePositionLock)) {
//            Log.d("DriveCommand", "Doing teleop drive");
            robot.follower.setTeleOpDrive(left_y, left_x, rotation,true);
//        }
//        robot.drivetrain.move(
//                direction,
//                rotation,
//                driveSpeed
//        );
    }

    @Override
    public void end(boolean interrupted) {

    }
    
    public double slr(double joystick_value) {
        return Math.pow(joystick_value, 5);
    }

    public double getHeadingError() {
        if (robot.follower.getCurrentPath() == null) {
            return 0;
        }

        return MathFunctions.getTurnDirection(robot.follower.getPose().getHeading(), HEADING_LOCK_ANGLE) *
                MathFunctions.getSmallestAngleDifference(robot.follower.getPose().getHeading(), HEADING_LOCK_ANGLE);
    }

}
