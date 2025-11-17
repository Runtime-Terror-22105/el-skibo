package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.teamcode.math.controllers.PidfController;
import org.firstinspires.ftc.teamcode.robot.hardware.sensors.TerrorAnalogEncoder;
import org.firstinspires.ftc.teamcode.robot.hardware.sensors.TerrorColorSensor;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.subsystems.vision.CameraSubsystem;

@Config
public class SpindexerSubsystem extends SubsystemBase {

    private enum COLOR {
        GREEN, PURPLE
    }



    private final RobotHardware hardware;
    private final Robot robot;

    public double spindexerOffset = 0;

    public static double TICKS_PER_REVOLUTION = ((1D+(46D/11D))*28D) * 5.6D;

    public static double INTAKE_WALL_1_ACTIVE = 1.0;
    public static double INTAKE_WALL_1_DEACTIVE = 0.0;
    public static double INTAKE_WALL_2_ACTIVE = 1.0;
    public static double INTAKE_WALL_2_DEACTIVE = 0.0;
    public static double SHOOTER_RAMP_ACTIVE = 0.3;
    public static double SHOOTER_RAMP_DEACTIVE = 0.0;
    public static double RESTING_SPINDEX_POS = 0.0;

    public static double spindexTransferPower=0.0;

    public double SHOOTER_INTAKE_SPEED = 0.0; // this is the speed where the shooter melonbotic servo intakes the balls

    public static double SHOOTER_INTAKING_SPEED = 1.0;
    public static double SHOOT_ONE_ROTATION = -(2/3)* Math.PI;

    public double intakeWallPosition1 = INTAKE_WALL_1_DEACTIVE;
    public double intakeWallPosition2 = INTAKE_WALL_2_DEACTIVE;
    public double shooterRampPosition = SHOOTER_RAMP_DEACTIVE;

    public static double DIDDY_POLE_ACTIVE = 0.6;

    public static double DIDDY_POLE_DEACITVE = 0.98;

    public double diddyPos= DIDDY_POLE_DEACITVE;

    public boolean usespindexPID=true;

    public double spindexerPower = 0.0;
    public TerrorColorSensor[] sensors;

    public boolean transferActive = false;

    public static double leftPosition =(4D/3D)* Math.PI;
    public static double rightPosition =(2D/3D)* Math.PI;
    public static double backPosition = 0.0;
    public static double readyPosition = (1D/6D)* Math.PI; //position for the first ball as the ramp goes down
    double[] yawOffsets = {0, (2.0 / 3) * Math.PI, -((2.0 / 3) * Math.PI)}; // todo: this is currently duplicate, make it so it just uses the above 3 variables

    public static PidfController.PidfCoefficients turningPidCoefficients =
            new PidfController.PidfCoefficients(0.011, 0, 0.00001, 0, 0);
    public static double yawPidTolerance = 0.1;
    private boolean pidEnabled = true;
    public final PidfController yawPid = new PidfController(turningPidCoefficients);

    public SpindexerSubsystem(RobotHardware hardware, Robot robot) {
        this.robot = robot;
        this.hardware = hardware;
        this.sensors  = new TerrorColorSensor[] {hardware.rightSensor, hardware.topSensor, hardware.leftSensor};

        this.yawPid.setTolerance(yawPidTolerance);
        this.yawPid.setTargetPosition(0.0);
    }

    public double getTargetYaw() {
        return this.yawPid.getTargetPosition();
    }

    public void setYaw(double angle) { //angle is in radians cuz i said so oh yeah and also have todo: optimization like the swerve pod thingy where u do the shortest distance
        this.yawPid.setTargetPosition(angle);
        this.usespindexPID=true;
    }

    public boolean getLimitSwitchState() {
        return !this.hardware.spindexerLimitSwitch.getState();
    }

    public void setPidEnabled(boolean enabled) {
        spindexerPower = 0.0;
        pidEnabled = enabled;
    }

    public void updateSpindexer(){
//        if(hardware.spindexerEncoder.getCurrentPosition())
        if (pidEnabled) {
            this.spindexerPower = yawPid.calculatePower(getPosition(), 0);
        }

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


    public void setWallActive(){
        this.intakeWallPosition1 = INTAKE_WALL_1_ACTIVE;
        this.intakeWallPosition2 = INTAKE_WALL_2_ACTIVE;
    }
    public void setWallDeactive(){
        this.intakeWallPosition1 = INTAKE_WALL_1_DEACTIVE;
        this.intakeWallPosition2 = INTAKE_WALL_2_DEACTIVE;
    }

    public void Oilup(){
        this.diddyPos= DIDDY_POLE_ACTIVE;
    }



    public void enableRamp(){
        shooterRampPosition=SHOOTER_RAMP_ACTIVE;
    }

    public void disableRamp(){
        shooterRampPosition=SHOOTER_RAMP_DEACTIVE;
    }

    public void setSpindexerOffset(double offset) {
        this.spindexerOffset = offset;
    }

    public void sortBalls(){
        double startPos = this.getPosition();
        int fullCount = 0;
        double greenPos = 0.0;
        int greenCount = 0;
        int purpleCount = 0;
        for (TerrorColorSensor sensor : this.sensors){
            if (sensor.getGreenOrPurple() != 'N'){
                fullCount += 1;
                if (sensor.getGreenOrPurple() == 'G'){
                    greenCount +=1;
                    if (sensor.position.equals(TerrorColorSensor.side.LEFT)){
                        greenPos = this.leftPosition;
                    }
                    else if (sensor.position.equals(TerrorColorSensor.side.RIGHT)){
                        greenPos = this.rightPosition;
                    }
                    else {
                        greenPos = this.backPosition;
                    }

                }
                else {
                    purpleCount += 1;
                }

            }
        }
        if (purpleCount == 2 && greenCount == 1){
            if (robot.camera.gameGlyph == CameraSubsystem.GLYPH.GPP) {
                double normalizedError = MathUtils.normalizeRadians((this.readyPosition-greenPos), true);
                if (normalizedError >= 0.1){
                    normalizedError = -((2* Math.PI) - normalizedError);
                }
                this.setYaw(startPos + normalizedError);

            }
            else if (robot.camera.gameGlyph == CameraSubsystem.GLYPH.PGP) {
                double normalizedError = MathUtils.normalizeRadians(((this.readyPosition-((2/3)*Math.PI))-greenPos), true);
                if (normalizedError >= 0.1){
                    normalizedError = -((2* Math.PI) - normalizedError);
                }
                this.setYaw(startPos + normalizedError);

            }
            else {
                double normalizedError = MathUtils.normalizeRadians(((this.readyPosition-((4/3)*Math.PI))-greenPos), true);
                if (normalizedError >= 0.1){
                    normalizedError = -((2* Math.PI) - normalizedError);
                }
                this.setYaw(startPos + normalizedError);
            }
        }
        else {
            this.setYaw(readyPosition);
        }

    }


    public void shootBall(){
        if (transferActive){
            this.setYaw(this.getPosition() + SHOOT_ONE_ROTATION);
        }
        else {
            this.shootBall();

        }


    }
    public void shootThree(){
        for (int i = 0; i < 3; i++){
            this.shootBall();
        }

    }

    public void setSpindexerPower(double power) {
        this.spindexerPower = power;
        this.usespindexPID=false;
    }

    @Override
    public void periodic() {
        //to the spindexers of Australia: Robot.camera.getBalls();
        //G:green P:purple N:none
        //0:top 1:right 2:left
        //returns char[]

//        this.hardware.spindexerIntakeRampServo1.setPosition(this.intakeRampPosition1);
//        this.hardware.spindexerIntakeRampServo2.setPosition(this.intakeRampPosition2);
//        this.hardware.spindexerTransferRampServo.setPosition(this.shooterRampPosition);
//        this.hardware.spindexerDiddyServo.setPosition(this.wallPosition);

        this.updateSpindexer();
        this.hardware.spindexerRotate.setPower(this.spindexerPower);
        this.hardware.spindexerIntakeWallServo1.setPosition(intakeWallPosition1);
        this.hardware.spindexerIntakeWallServo2.setPosition(intakeWallPosition2);
        this.hardware.spindexerDiddyServo.setPosition(diddyPos);
        this.hardware.spindexerTransferRampServo.setPosition(shooterRampPosition);

    }
}
