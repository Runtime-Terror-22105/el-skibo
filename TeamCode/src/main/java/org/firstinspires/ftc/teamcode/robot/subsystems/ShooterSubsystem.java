package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.math.controllers.PidfController;

public class ShooterSubsystem extends SubsystemBase {
    private final RobotHardware hardware;

    public static PidfController.PidfCoefficients shooterPIDCoeffecients =
            new PidfController.PidfCoefficients(0.0, 0, 0.00, 0, 0);
    public static double shooterVelocityTolerance = 0.0;

    public double hoodPosition=0.0;

    public final PidfController shooterPID = new PidfController(shooterPIDCoeffecients);
    public double shooterSpeed=0.0;

    public ShooterSubsystem(RobotHardware hardware) {

        this.hardware = hardware;

        this.shooterPID.setTolerance(this.shooterVelocityTolerance);
        this.shooterPID.setTargetPosition(0.0);



    }


    public void setSpeed(double goal){
        this.shooterPID.setTargetPosition(goal);
    }

    public double getShooterVelocity(){
        return this.hardware.shooterEncoder.getVelocity();
    }

    public void updateShooter(){
        this.shooterSpeed=this.shooterPID.calculatePower(this.getShooterVelocity(),0);
    }

    public void setHoodPosition(double Position){
        this.hoodPosition=Position;
    }







    @Override
    public void periodic() {
        // TODO: do something with hardware.shooter here...
        hardware.hood.setPosition(this.hoodPosition);
        this.updateShooter();
        hardware.shooter.setPower(shooterSpeed);
    }
}
