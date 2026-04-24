package org.firstinspires.ftc.teamcode.robot.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;
import org.firstinspires.ftc.teamcode.util.Profiler;

@Config
public class IntakeSubsystem extends SubsystemBase {
    public static double DEFAULT_SPEED = 1.0;
    public static double REVERSE_SPEED = -0.8;

    public static boolean debug = false;

    public static double INTAKE_LEFT_DOWN = 0.53;
    public static double INTAKE_LEFT_UP = 0.76;

    public static double INTAKE_RIGHT_DOWN = 0.45;
    public static double INTAKE_RIGHT_UP = 0.22;

    private boolean intakeLifted = false;

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

    public void setIntakeLifted(boolean isLifted)
    {
        this.intakeLifted = isLifted;
    }

    /**
     * i saw aadits face
     * in a crowded place
     * and i dont know what to do
     * cause ill never pee pee poo :(
     * (if the repo goes public its actually over for me)
     */
    public boolean getIntakeLifted()
    {
        return this.intakeLifted;
    }

    @Override
    public void periodic() {
        try (Profiler.Scope p = Profiler.enter("IntakeSubsystem")) {
            if (robot.robotState.isHang()) {
                robot.hardware.colorSensors.setUpdatePeriod(100);
                robot.hardware.intake.setPower(0);
                return;
            }

            double leftEffective = INTAKE_LEFT_DOWN;
            double rightEffective = INTAKE_RIGHT_DOWN;

            if(intakeLifted)
            {
                leftEffective = INTAKE_LEFT_UP;
                rightEffective = INTAKE_RIGHT_UP;
            }

            robot.hardware.intakeServoLeft.setPosition(leftEffective);
            robot.hardware.intakeServoRight.setPosition(rightEffective);

            // When not intaking, increase the update period to reduce I2C load
            robot.hardware.colorSensors.setUpdatePeriod(RobotState.INTAKING.equals(robot.robotState) ? 1 : 1);
            robot.hardware.intake.setPower(this.targetSpeed);
            if (debug) {
                Log.i("IntakeSubsystem", "Intake motor power: " + robot.hardware.intake.getPower());
            }
        }
    }
}
