package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.math.controllers.PidfController;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@Config
public class SpindexerSubsystem extends SubsystemBase {

    private enum COLOR {
        GREEN, PURPLE
    }

    private final RobotHardware hardware;

    public double spindexerOffset = 0;

    public static double INTAKE_RAMP_1_ACTIVE = 1.0;
    public static double INTAKE_RAMP_1_DEACTIVE = 0.0;
    public static double INTAKE_RAMP_2_ACTIVE = 1.0;
    public static double INTAKE_RAMP_2_DEACTIVE = 0.0;
    public static double SHOOTER_RAMP_ACTIVE = 1.0;
    public static double SHOOTER_RAMP_DEACTIVE = 0.0;
    public static double WALL_ACTIVE = 1.0;
    public static double WALL_DEACTIVE = 0.0;
    public static double RESTING_SPINDEX_POS = 0.0;

    public double SHOOTER_INTAKE_SPEED = 0.0; // this is the speed where the shooter melonbotic servo intakes the balls

    public static double SHOOTER_INTAKING_SPEED = 1.0;
    public static double SHOOT_ONE_ROTATION = -(2/3)* Math.PI;

    public double intakeRampPosition1 = INTAKE_RAMP_1_DEACTIVE;
    public double intakeRampPosition2 = INTAKE_RAMP_2_DEACTIVE;
    public double shooterRampPosition = SHOOTER_RAMP_DEACTIVE;
    public double wallPosition = WALL_DEACTIVE;

    public double spindexerPower = 0.0;

    public static double leftPosition =(4D/3D)* Math.PI;
    public static double rightPosition =(2D/3D)* Math.PI;
    public static double backPosition = 0.0;
    public static double readyPosition = (1D/6D)* Math.PI; //position for the first ball as the ramp goes down
    double[] yawOffsets = {0, (2.0 / 3) * Math.PI, -((2.0 / 3) * Math.PI)}; // todo: this is currently duplicate, make it so it just uses the above 3 variables


    public enum position {
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

    public double getTargetYaw() {
        return this.yawPid.getTargetPosition();
    }

    public void setYaw(double angle) { //angle is in radians cuz i said so oh yeah and also have todo: optimization like the swerve pod thingy where u do the shortest distance
        this.yawPid.setTargetPosition(angle);
    }

    public boolean getLimitSwitchState() {
        return this.hardware.spindexerLimitSwitch.getState();
    }

    public void updateSpindexer(){
//        if(hardware.spindexerEncoder.getCurrentPosition())
        this.spindexerPower= yawPid.calculatePower(hardware.spindexerEncoder.getCurrentPosition()+this.spindexerOffset,0);
        // setting pid power into the spindexer
    }

    public char[] getBallPositions()
    {
        return new char[]{hardware.topSensor.getGreenOrPurple(), hardware.rightSensor.getGreenOrPurple(), hardware.leftSensor.getGreenOrPurple()};
    }

    public void selectColor(char color)
    {
        int nearestIndex = new String(getBallPositions()).indexOf(color);
        if(nearestIndex == -1)
        {
            //TODO: add error handling here some telemetry message abt not having balls or smth
            return;
        }
        setYaw(this.yawPid.getTargetPosition() + yawOffsets[nearestIndex]);
    }

    public double getPosition() {
        return hardware.spindexerEncoder.getCurrentPosition() - this.spindexerOffset;
    }

    public void activateTransfer() {
        //this.initShootPos();
        this.intakeRampPosition1 = INTAKE_RAMP_1_ACTIVE;
        this.intakeRampPosition2 = INTAKE_RAMP_2_ACTIVE;
        this.shooterRampPosition = SHOOTER_RAMP_ACTIVE;
        this.wallPosition = WALL_ACTIVE;
        this.SHOOTER_INTAKE_SPEED = SHOOTER_INTAKING_SPEED;
    }

    public void deactivateTransfer() {
        this.intakeRampPosition1 = INTAKE_RAMP_1_DEACTIVE;
        this.intakeRampPosition2 = INTAKE_RAMP_2_DEACTIVE;
        this.shooterRampPosition = SHOOTER_RAMP_DEACTIVE;
        this.wallPosition = WALL_DEACTIVE;
        this.SHOOTER_INTAKE_SPEED = 0.0;
        this.setYaw(this.getPosition()-((1/6)*Math.PI));

    }

    public void setSpindexerOffset(double offset) {
        this.spindexerOffset = offset;
    }

//    public void initShootPos(){
//        double startPos = this.getPosition();
//        int fullCount = 0;
//        double greenPos;
//        int greenCount = 0;
//        int purpleCount = 0;
//        for (ColorSensor sensor in this.colorSensors){
//            if (sensor == has ball){
//                fullCount += 1;
//                if (sensor.color == green){
//                    greenCount +=1;
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
//                else {
//                    purpleCount += 1;
//                }
//
//            }
//        }
//        if (purpleCount == 2 && greenCount == 1){
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
//        else {
//            this.setYaw(readyPosition);
//        }
//
//    }


    public void shootBall(){
        // TODO: Chack if transfer is down before running
        this.setYaw(this.getPosition() + SHOOT_ONE_ROTATION);

    }
    public void shootThree(){
        for (int i = 0; i < 3; i++){
            this.shootBall();
        }

    }

    public void setSpindexerPower(double power) {
        this.spindexerPower = power;
    }

    @Override
    public void periodic() {
        //to the spindexers of Australia: Robot.camera.getBalls();
        //G:green P:purple N:none
        //0:top 1:right 2:left
        //returns char[]

        this.hardware.spindexerIntakeRampServo1.setPosition(this.intakeRampPosition1);
        this.hardware.spindexerIntakeRampServo2.setPosition(this.intakeRampPosition2);
        this.hardware.spindexerTransferRampServo.setPosition(this.shooterRampPosition);
        this.hardware.spindexerDiddyServo.setPosition(this.wallPosition);

        this.updateSpindexer();
        this.hardware.spindexerRotate.setPower(this.spindexerPower);
    }
}
