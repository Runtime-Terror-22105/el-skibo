package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

public class IntakeSubsystem extends SubsystemBase {
    private final RobotHardware hardware;

    public static double defaultDown = 0.5; //servo pos
    public static double defaultUp = 0.75;
    public static double defaultSpeed = 0.9;

    private double targetPitch;
    private double targetSpeed;

    public IntakeSubsystem(RobotHardware hardware) {
        this.hardware = hardware;
        this.targetPitch = defaultUp;
        this.targetSpeed = 0;
    }

    public void putDown(){
        this.targetPitch = defaultDown;
    }
    public void putUp(){
        this.targetPitch = defaultUp;
    }
    public void setPitch(double pitch){
        this.targetPitch = pitch;
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
        return this.targetPitch;
    }
    public double getSpeed(){
        return this.targetSpeed;
    }


    @Override
    public void periodic() {
        hardware.intake.setPower(this.targetSpeed);
        hardware.intakePitch.setPosition(this.targetPitch);
    }
}
