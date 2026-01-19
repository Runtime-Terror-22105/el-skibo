package org.firstinspires.ftc.teamcode.robot.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;
import org.firstinspires.ftc.teamcode.util.Profiler;

@Config
public class IntakeSubsystem extends SubsystemBase {
    public static double DEFAULT_SPEED = 0.9;

    private final Robot robot;
    private double targetSpeed;

    public IntakeSubsystem(Robot robot) {
        this.robot = robot;
        this.targetSpeed = 0;
    }

    public void setSpeed(double speed){
        this.targetSpeed = speed;
    }

    public double getSpeed(){
        return this.targetSpeed;
    }

    @Override
    public void periodic() {
        try (Profiler.Scope p = Profiler.enter("IntakeSubsystem")) {
            robot.hardware.setEnableColorSensor(RobotState.INTAKING.equals(robot.robotState));
            robot.hardware.intake.setPower(this.targetSpeed);
            Log.i("IntakeSubsystem", "Intake motor power: " + robot.hardware.intake.getPower());
        }
    }
}
