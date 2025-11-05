package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.MathUtils;
import org.firstinspires.ftc.teamcode.math.controllers.PidfController;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

public class SpindexerSubsystem extends SubsystemBase {
    private final RobotHardware hardware;

    public static double activatePosition=1.0; // cams up

    public static double deactivatePosition=0.0; // cams down

    public double SHOOTER_INTAKE_SPEED=0.0; // this is the speed where the shooter melonbotic servo intakes the balls

    public static double SHOOTER_INTAKING_SPEED=1.0;

    public static double leftPosition =(4/3)* Math.PI;
    public static double rightPosition =(2/3)* Math.PI;
    public static double backPosition = 0.0;
    public static double readyPosition = (1/6)* Math.PI; //position for the first ball as the ramp goes down


    public double PopperPosition=deactivatePosition;


    public double spindexerPower=0.0;
    public enum position{
        LEFT,
        RIGHT,
        BACK

    }
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

    public void activateTransfer(){
        this.PopperPosition=this.activatePosition;
        this.SHOOTER_INTAKE_SPEED=this.SHOOTER_INTAKING_SPEED;

    }

    public void deactivateTransfer(){
        this.PopperPosition=this.deactivatePosition;
        this.SHOOTER_INTAKE_SPEED=0.0;
    }
//
//    public void initMotifPos(){
//        double startPos = this.getPosition();
//        int fullCount = 0;
//        double greenPos;
//        for (ColorSensor sensor in this.colorSensors){
//            if (sensor == has ball){
//                fullCount += 1;
//                if (sensor.color == green){
//                    if (sensor.pos == position.LEFT){
//                        greenPos = this.leftPosition;
//                    }
//                    else if (sensor.pos == position.RIGHT){
//                        greenPos = this.rightPosition;
//                    }
//                    else {
//                        greenPos = this.backPosition;
//                    }
//
//                }
//            }
//        }
//        if (fullCount == 3){
//            if (motif == GPP) {
//                double normalizedError = MathUtils.normalizeRadians((this.readyPosition-greenPos), true);
//                if (normalizedError >= 0.1){
//                    normalizedError = -((2* Math.PI) - normalizedError);
//                }
//                this.setYaw(startPos + normalizedError);
//
//            }
//            else if (motif == PGP) {
//                double normalizedError = MathUtils.normalizeRadians(((this.readyPosition-((2/3)*Math.PI))-greenPos), true);
//                if (normalizedError >= 0.1){
//                    normalizedError = -((2* Math.PI) - normalizedError);
//                }
//                this.setYaw(startPos + normalizedError);
//
//            }
//            else {
//                double normalizedError = MathUtils.normalizeRadians(((this.readyPosition-((4/3)*Math.PI))-greenPos), true);
//                if (normalizedError >= 0.1){
//                    normalizedError = -((2* Math.PI) - normalizedError);
//                }
//                this.setYaw(startPos + normalizedError);
//            }
//        }
//
//    }
//




    @Override
    public void periodic() {
        this.hardware.spindexerCamPopper.setPosition(this.PopperPosition);
        this.updateSpindexer();
        this.hardware.spindexerRotate.setPower(this.spindexerPower);
    }
}
