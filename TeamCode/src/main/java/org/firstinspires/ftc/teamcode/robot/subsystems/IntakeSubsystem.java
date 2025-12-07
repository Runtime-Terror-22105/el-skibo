package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake.IntakePitch;

@Config
public class IntakeSubsystem extends SubsystemBase {
    private final RobotHardware hardware;

    public static double DOWN_LEFT = 0.61; //servo pos
    public static double UP_LEFT = 0.2;
    public static double DOWN_RIGHT = 0.045;
    public static double UP_RIGHT = 0.44;

    public static double DEFAULT_SPEED = 0.9;

    private double targetSpeed;

    //Down is intaking, up is resting

    //Two states: up and down, with keys 'left' and 'right' which have the respective positions for the servos
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

    public void setSpeed(double speed){
        this.targetSpeed = speed;
    }

    public double getSpeed(){
        return this.targetSpeed;
    }

    @Override
    public void periodic() {
        hardware.intake.setPower(this.targetSpeed);
        hardware.intakePitchLeft.setPosition(pitch.left.get());
        hardware.intakePitchRight.setPosition(pitch.right.get());
    }
}
