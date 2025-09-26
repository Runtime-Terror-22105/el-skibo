package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

public class IntakeSubsystem extends SubsystemBase {
    private final RobotHardware hardware;

    public static double defaultDown = 0.5; //servo pos
    public static double defaultUp = 0.75;
    public static double defaultSpeed = 0.9;

    private double targetPos;
    private double targetSpeed;

    public IntakeSubsystem(RobotHardware hardware) {
        this.hardware = hardware;
        this.targetPos = defaultUp;
        this.targetSpeed = 0;
    }
    public void putDown(){
        this.targetPos = defaultDown;
    }
    public void putUp(){
        this.targetPos = defaultUp;
    }
    public void setPos(double pos){
        this.targetPos = pos;
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
    public double getPos(){
        return this.targetPos;
    }
    public double getSpeed(){
        return this.targetSpeed;
    }


    @Override
    public void periodic() {
        hardware.intake.setPower(this.targetSpeed);
        hardware.intakePitch.setPosition(this.targetPos);
        // TODO: do something with hardware.intake here...
    }
}
