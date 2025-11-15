package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@Config
public class IntakeSubsystem extends SubsystemBase {
    private final RobotHardware hardware;

    public static double defaultDown1 = 0.05; //servo pos
    public static double defaultDown2 = 1.0;
    public static double defaultUp1 = 0.2;
    public static double defaultUp2 = 0.84;
    public static double defaultSpeed = 0.9;

    public boolean isUp = false;

    private double targetPitch1;
    private double targetPitch2;
    private double targetSpeed;


    public IntakeSubsystem(RobotHardware hardware) {
        this.hardware = hardware;
        this.targetPitch1 = defaultUp1;
        this.targetPitch2 = defaultUp2;
        this.targetSpeed = 0;
    }

    public void putDown(){
        this.isUp = false;
        this.targetPitch1 = defaultDown1;
        this.targetPitch2 = defaultDown2;
    }
    public void putUp(){
        this.isUp = true;
        this.targetPitch1 = defaultUp1;
        this.targetPitch2 = defaultUp2;
    }
    public void setPitch(double pitch){
        this.targetPitch1 = pitch;
        this.targetPitch2 = 0.5 - (0.5 - pitch);
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
        return this.targetPitch1;
    }
    public double getSpeed(){
        return this.targetSpeed;
    }

    public void setIntakePitchPosition()
    {
        hardware.intakePitch1.setPosition(targetPitch1);
        hardware.intakePitch2.setPosition(targetPitch2);
    }


    @Override
    public void periodic() {
        hardware.intake.setPower(this.targetSpeed);
        setIntakePitchPosition();
    }
}
