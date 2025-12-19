package org.firstinspires.ftc.teamcode.robot.command.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;
import org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem;

@Config
public class ShimmyShoot3BallAlt extends CommandBase {
    private ElapsedTime timer = new ElapsedTime();

    private Robot robot;

    public ShimmyShoot3BallAlt(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void initialize()
    {
        robot.robotState = RobotState.SHOOTING;
        robot.intake.setSpeed(IntakeSubsystem.DEFAULT_SPEED);
        timer.reset();
    }

    @Override
    public void execute() {

    }

//    @Override
//    public boolean isFinished()
//    {
//        return false;
//    }


    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        robot.spindexer.setSpindexerPower(0.0);
        robot.spindexer.setPidEnabled(true);
        robot.intake.setSpeed(0);
        robot.spindexer.deactivatePole();
        robot.spindexer.disableRamp();
        robot.spindexer.goToAngle120(0);
        robot.robotState = RobotState.RESTING;
    }
}