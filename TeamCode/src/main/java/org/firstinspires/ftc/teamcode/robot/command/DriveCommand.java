package org.firstinspires.ftc.teamcode.robot.command;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.Subsystem;

import java.util.Collections;
import java.util.Set;
import java.util.function.BooleanSupplier;

import org.firstinspires.ftc.teamcode.math.Coordinate;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import java.util.function.Supplier;

public class DriveCommand extends CommandBase {

    Robot robot = new Robot();
    public static double driveSpeed = 0.9;

    private final Supplier<Double> x, y, turn;

    public DriveCommand(
            Supplier<Double> x,
            Supplier<Double> y,
            Supplier<Double> turn) {
        this.x = x;
        this.y = y;
        this.turn = turn;

        super.addRequirements(robot.drivetrain);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        //add more stuff from before i dont want to do this
        Coordinate direction = new Coordinate(x.get(), y.get());
        double rotation = turn.get();
        robot.drivetrain.move(
                direction,
                rotation,
                driveSpeed
        );
    }

    @Override
    public void end(boolean interrupted) {

    }

//    @Override
//    public Set<Subsystem> getRequirements() {
//        return Collections.emptySet();
//    }
}
