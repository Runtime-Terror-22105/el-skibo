package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake.IntakePitch;

@Config
public class IntakeSubsystem extends SubsystemBase {
    private final RobotHardware hardware;

    public static double DOWN_LEFT = 0.91; //servo pos
    public static double UP_LEFT = 0.41;
    public static double DOWN_RIGHT = 0.0;
    public static double UP_RIGHT = 0.5;

    public static double MIN_LEFT = 0.41;
    public static double MAX_LEFT = 0.91; //servo pos
    public static double MIN_RIGHT = 0.0;
    public static double MAX_RIGHT = 0.5;

    public static double DEFAULT_SPEED = 0.9;

    private double targetSpeed;

    private IntakePitch pitch = IntakePitch.DOWN;

    public IntakeSubsystem(RobotHardware hardware) {
        this.hardware = hardware;
        this.targetSpeed = 0;
    }

    public void setPitch(IntakePitch pitch) {
        this.pitch = pitch;
    }

    public IntakePitch getPitch() {
        return pitch;
    }

    public void turnOn(){
        this.targetSpeed = DEFAULT_SPEED;
    }

    public void turnOff() {
        this.targetSpeed = 0.0;
    }

    public void setSpeed(double speed){
        this.targetSpeed = speed;
    }

    public double getSpeed(){
        return this.targetSpeed;
    }

    @Override
    public void periodic() {
        hardware.intake.setPower(this.targetSpeed);
        hardware.intakePitchLeft.setPosition(Math.max(MIN_LEFT, Math.min(MAX_LEFT, pitch.left.get())));
        hardware.intakePitchRight.setPosition(Math.max(MIN_RIGHT, Math.min(MAX_RIGHT, pitch.right.get())));
    }
}
