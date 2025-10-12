package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.math.controllers.PidfController;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

import java.util.Arrays;
import java.util.Collections;

public class SpindexerSubsystem extends SubsystemBase {

    private enum COLOR
    {
        GREEN,PURPLE
    }
    double[] yawOffsets = {0, (2.0 / 3) * Math.PI, -((2.0 / 3) * Math.PI)};

    private final RobotHardware hardware;
    public static double activatePosition=1.0; // cams up

    public static double deactivatePosition=0.0; // cams down

    public double SHOOTER_INTAKE_SPEED=0.0; // this is the speed where the shooter melonbotic servo intakes the balls

    public static double SHOOTER_INTAKING_SPEED=1.0;

    public double PopperPosition=deactivatePosition;

    public double spindexerPower=0.0;
    public static PidfController.PidfCoefficients turningPidCoefficients =
            new PidfController.PidfCoefficients(0.014, 0, 0, 1, 0);
    public static double yawPidTolerance = 0.1;
    public final PidfController yawPid = new PidfController(turningPidCoefficients);

    public SpindexerSubsystem(RobotHardware hardware) {
        this.hardware = hardware;

        this.yawPid.setTolerance(yawPidTolerance);
        this.yawPid.setTargetPosition(0.0);
    }

    public void setYaw(double angle){ //angle is in radians cuz i said so oh yeah and also have todo: optimization like the swerve pod thingy where u do the shortest distance
        this.yawPid.setTargetPosition(angle);
    }

    public void updateSpindexer(){
//        if(hardware.spindexerEncoder.getCurrentPosition())
        this.spindexerPower= yawPid.calculatePower(hardware.spindexerEncoder.getCurrentPosition(),0);
        // setting pid power into the spindexer
    }

    public double getPosition(){
        return hardware.spindexerEncoder.getCurrentPosition();
    }

    public void selectColor(char color)
    {
        int nearestIndex = new String(Robot.camera.getBalls()).indexOf(color);
        if(nearestIndex == -1)
        {
            //TODO: add error handling here some telemetry message abt not having balls or smth
            return;
        }
        setYaw(this.yawPid.getTargetPosition() + yawOffsets[nearestIndex]);
    }

    public void activateTransfer(){
        this.PopperPosition=this.activatePosition;
        this.SHOOTER_INTAKE_SPEED=this.SHOOTER_INTAKING_SPEED;

    }

    public void deactivateTransfer(){
        this.PopperPosition=this.deactivatePosition;
        this.SHOOTER_INTAKE_SPEED=0.0;
    }

    @Override
    public void periodic() {

        //to the spindexers of Australia: Robot.camera.getBalls();
        //G:green P:purple N:none
        //0:top 1:right 2:left
        //returns char[]

        this.hardware.spindexerCamPopper.setPosition(this.PopperPosition);
        this.updateSpindexer();
        this.hardware.spindexerRotate.setPower(this.spindexerPower);
    }
}
