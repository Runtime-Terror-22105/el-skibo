package org.firstinspires.ftc.teamcode.robot.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake.IntakePitch;
import org.firstinspires.ftc.teamcode.util.Profiler;

@Config
public class IntakeSubsystem extends SubsystemBase {
    private final Robot robot;

    public static double DOWN_LEFT = 0.57; //servo pos
    public static double UP_LEFT = 0.14;
    public static double DOWN_RIGHT = 0.03;
    public static double UP_RIGHT = 0.44;

    public static double DEFAULT_SPEED = 0.9;

    private double targetSpeed;

    //Down is intaking, up is resting

    //Two states: up and down, with keys 'left' and 'right' which have the respective positions for the servos
    private IntakePitch pitch = IntakePitch.DOWN;

    public IntakeSubsystem(Robot robot) {
        this.robot = robot;
        this.targetSpeed = 0;
    }

    public void setPitch(IntakePitch pitch) {
        this.pitch = pitch;
    }

    public IntakePitch getPitch() {
        return pitch;
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
            robot.hardware.intakePitchLeft.setPosition(pitch.left.get());
            robot.hardware.intakePitchRight.setPosition(pitch.right.get());

            Log.i("IntakeSubsystem", "Intake motor power: " + robot.hardware.intake.getPower());
        }
    }
}
