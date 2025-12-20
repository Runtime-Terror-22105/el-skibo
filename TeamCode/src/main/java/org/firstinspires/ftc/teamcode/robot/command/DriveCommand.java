package org.firstinspires.ftc.teamcode.robot.command;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.Subsystem;

import java.util.Collections;
import java.util.Set;
import java.util.function.BooleanSupplier;

import org.firstinspires.ftc.teamcode.math.Algebra;
import org.firstinspires.ftc.teamcode.math.Coordinate;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import java.util.function.Supplier;

@Config
public class DriveCommand extends CommandBase {

    public static double ROTATION_MULTIPLIER = 0.5;

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

        super.addRequirements(robot.drive);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        //add more stuff from before i dont want to do this

        double deadzone_amt = 0;
        double left_x = -x.get();
        double left_y = -y.get();
        double right_x = -turn.get();
        left_x = Math.signum(left_x) * Algebra.mapRange(Math.abs(left_x), deadzone_amt, 1.0, 0.0, 1.0);
        left_y = Math.signum(left_y) * Algebra.mapRange(Math.abs(left_y), deadzone_amt, 1.0, 0.0, 1.0);
        right_x = Math.signum(right_x) * Algebra.mapRange(Math.abs(right_x), deadzone_amt, 1.0, 0.0, 1.0);

        robot.follower.setTeleOpDrive(left_y, left_x, right_x*ROTATION_MULTIPLIER);
//        robot.drivetrain.move(
//                direction,
//                rotation,
//                driveSpeed
//        );
    }

    @Override
    public void end(boolean interrupted) {

    }

//    @Override
//    public Set<Subsystem> getRequirements() {
//        return Collections.emptySet();
//    }

    public double slr(double joystick_value) {
        return Math.pow(joystick_value, 5);
    }

}
