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

    public static double ROTATION_MULTIPLIER_14V = 0.4315;
    public static double ROTATION_MULTIPLIER_12V = 0.515;

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
        double rotationMultiplier = Algebra.mapRangeNoClamp(robot.hardware.initialVoltage, 12, 14, ROTATION_MULTIPLIER_12V, ROTATION_MULTIPLIER_14V);
        double left_x = -x.get();
        double left_y = -y.get();
        double right_x = -turn.get();
        robot.follower.setTeleOpDrive(left_y, left_x, right_x * rotationMultiplier);
    }

    @Override
    public void end(boolean interrupted) {

    }
    
    public double slr(double joystick_value) {
        return Math.pow(joystick_value, 5);
    }

}
