package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@Config
public class IntakeSubsystem extends SubsystemBase {
    private final RobotHardware hardware;

    public static double defaultDownLeft = 0.84; //servo pos
    public static double defaultDownRight = 0.19;
    public static double defaultUpLeft = 0.4;
    public static double defaultUpRight = 0.6;
    public static double defaultSpeed = 0.9;

    public boolean isUp = false;

    private double targetPitchLeft;
    private double targetPitchRight;
    private double targetSpeed;


    public IntakeSubsystem(RobotHardware hardware) {
        this.hardware = hardware;
        this.targetPitchLeft = defaultUpLeft;
        this.targetPitchRight = defaultUpRight;
        this.targetSpeed = 0;
    }

    public void putDown(){
        this.isUp = false;
        this.targetPitchLeft = defaultDownLeft;
        this.targetPitchRight = defaultDownRight;
    }
    public void putUp(){
        this.isUp = true;
        this.targetPitchLeft = defaultUpLeft;
        this.targetPitchRight = defaultUpRight;
    }
    public void setPitch(double pitch){
        this.targetPitchLeft = pitch;
        this.targetPitchRight = 0.5 - (0.5 - pitch);
    }

    public void turnOn(){
        this.targetSpeed = defaultSpeed;
    }
    public void turnOff(){
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

    public void setIntakePitchPosition()
    {
        hardware.intakePitchLeft.setPosition(targetPitchLeft);
        hardware.intakePitchRight.setPosition(targetPitchRight);
    }


    @Override
    public void periodic() {
        hardware.intake.setPower(this.targetSpeed);
        setIntakePitchPosition();
    }
}
