package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@Config
public class IntakeSubsystem extends SubsystemBase {
    private final RobotHardware hardware;

    public static double PITCH_DOWN_LEFT = 0.91; //servo pos
    public static double PITCH_UP_LEFT = 0.41;
    public static double PITCH_DOWN_RIGHT = 0.0;
    public static double PITCH_UP_RIGHT = 0.5;

    public static double DEFAULT_SPEED = 0.9;

    private double targetPitchLeft;
    private double targetPitchRight;
    private double targetSpeed;


    public IntakeSubsystem(RobotHardware hardware) {
        this.hardware = hardware;
        this.targetPitchLeft = PITCH_UP_LEFT;
        this.targetPitchRight = PITCH_UP_RIGHT;
        this.targetSpeed = 0;
    }

    public void putDown(){
        this.targetPitchLeft = PITCH_DOWN_LEFT;
        this.targetPitchRight = PITCH_DOWN_RIGHT;
    }

    public void putUp(){
        this.targetPitchLeft = PITCH_UP_LEFT;
        this.targetPitchRight = PITCH_UP_RIGHT;
    }

    public boolean isUp() {
        return this.targetPitchLeft == PITCH_UP_LEFT && this.targetPitchRight == PITCH_UP_RIGHT;
    }

    public void setPitch(double pitch) {
        // TODO: this method should map each of the ranges to 0-1
//        this.targetPitchLeft = pitch;
//        this.targetPitchRight = 0.5 - (0.5 - pitch);
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

    public double getPitch(){
        return this.targetPitchLeft;
    }

    public double getSpeed(){
        return this.targetSpeed;
    }

    @Override
    public void periodic() {
        hardware.intake.setPower(this.targetSpeed);
        hardware.intakePitchLeft.setPosition(targetPitchLeft);
        hardware.intakePitchRight.setPosition(targetPitchRight);
    }
}
