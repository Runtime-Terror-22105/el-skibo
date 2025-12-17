package org.firstinspires.ftc.teamcode.robot.command.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.robocol.Command;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.command.intake.SetIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.SetSpindexerPoleActive;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.SetSpindexerRampActive;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.SetSpindexerYawCommand;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;
import org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem;

@Config
public class ShimmyShoot3Ball extends CommandBase {
//    public static double SPINDEX_ROTATIONS = -4.5;  // revolutions, negative bc clockwise
    public static double SPINDEX_TRANSFER_POWER = -1;
    public static int spindexTimeDelay = 200;

    public static int angleDiff = 5;

    private boolean setupFinished = false;

    private double lastKnownSpindexerPos = 0;

    private int ballsShot = 0;

    private ElapsedTime timer = new ElapsedTime();

    private Robot robot;

    public ShimmyShoot3Ball(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void initialize()
    {
        robot.robotState = RobotState.SHOOTING;
        robot.intake.setSpeed(IntakeSubsystem.DEFAULT_SPEED);
        timer.reset();
        robot.spindexer.goToAngle120(Math.toRadians(angleDiff));
        lastKnownSpindexerPos = 0;
        robot.spindexer.setPidEnabled(false);
    }

    @Override
    public void execute() {
        if(!setupFinished)
        {
            robot.spindexer.goToAngle120(robot.spindexer.getPosition()-Math.toRadians(angleDiff));
            setupFinished = timer.milliseconds() <= spindexTimeDelay;
            if(setupFinished)
            {
                timer.reset();
            }
            return;
        }
        if(Math.abs(robot.spindexer.getPosition()-lastKnownSpindexerPos) >= Math.toRadians(120))
        {
            robot.spindexer.setSpindexerPower(0);
            robot.spindexer.setPidEnabled(true);
            robot.spindexer.goToAngle120(lastKnownSpindexerPos+Math.toRadians(120+angleDiff));
            lastKnownSpindexerPos = robot.spindexer.getPosition();
            setupFinished = false;
            ballsShot++;
            timer.reset();
        }
        else
        {
            robot.spindexer.setPidEnabled(false);
            robot.spindexer.setSpindexerPower(SPINDEX_TRANSFER_POWER);
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
        robot.spindexer.setPidEnabled(true);
        robot.intake.setSpeed(0);
        robot.spindexer.Oildown();
        robot.spindexer.disableRamp();
        robot.spindexer.goToAngle120(0);
        robot.robotState = RobotState.RESTING;
    }
}