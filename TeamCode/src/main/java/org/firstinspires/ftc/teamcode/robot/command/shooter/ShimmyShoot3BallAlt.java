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

    public static double spindexTurnPower = -0.5;

    public static double turnTimeMS = 2000;

    public static double shootTimeMS = 2000;
    public static double shimmyTimeMS = 250;

    private double shimmyTarget = 0;
    private double unshimmyTarget = 0;

    private boolean isTurning = false;

    private boolean isShooting = false;

    private boolean isShimmying = false;

    private int ballsShot = 0;

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
        isShooting = true;
        robot.spindexer.setPidEnabled(true);
    }

    @Override
    public void execute() {
        if(isTurning)
        {
            isTurning = (timer.milliseconds() <= turnTimeMS);
            if(!isShooting)
            {
                timer.reset();
                isShooting = true;
            }
            return;
        }
        if(isShooting)
        {
            isShooting = (timer.milliseconds() <= shootTimeMS);
            robot.spindexer.setPidEnabled(false);
            robot.spindexer.setSpindexerPower(spindexTurnPower);
            if(!isShooting)
            {
                ballsShot++;
                isShimmying = true;
                timer.reset();
                robot.spindexer.setPidEnabled(true);
                robot.spindexer.setSpindexerPower(0);
                shimmyTarget = robot.spindexer.getPosition() + Math.toRadians(5);
                unshimmyTarget = robot.spindexer.getPosition();
            }
            return;
        }
        if(isShimmying)
        {
            if(timer.milliseconds() <= shimmyTimeMS/2)
            {
                robot.spindexer.goToAngle120(shimmyTarget);
            }
            if(timer.milliseconds() > shimmyTimeMS/2 && timer.milliseconds() <= shimmyTimeMS)
            {
                robot.spindexer.goToAngle120(unshimmyTarget);
            }
            isShimmying = (timer.milliseconds() <= shimmyTimeMS);
            if(!isShimmying)
            {
                robot.spindexer.rotate(Math.toRadians(120));
                isTurning = true;
                timer.reset();
            }
        }
    }

    @Override
    public boolean isFinished()
    {
        return ballsShot == 3;
    }


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