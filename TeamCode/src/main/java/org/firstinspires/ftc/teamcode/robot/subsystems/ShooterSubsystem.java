package org.firstinspires.ftc.teamcode.robot.subsystems;

import static org.firstinspires.ftc.teamcode.robot.subsystems.ShotType.Arc;
import static org.firstinspires.ftc.teamcode.robot.subsystems.ShotType.Straight;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.math.Algebra;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.math.controllers.PidfController;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;

@Config
public class ShooterSubsystem extends SubsystemBase {
    private final RobotHardware hardware;

    public static double TICKS_PER_REV = 28; // GoBilda yellowjacket encoder

    // TODO: tune velocity pid coefficients + tolerance
    public static PidfController.PidfCoefficients shooterPIDCoeffecients =
            new PidfController.PidfCoefficients(0.0001, 0.000115, 0.00, 0, 0);
    public static double shooterVelocityTolerance = 0.0;

    // the current pid + speed
    public final PidfController shooterPID = new PidfController(shooterPIDCoeffecients);
    public double shooterSpeed=0.0;

    // the current shooting angle
    public double hoodPosition= 0.0;
    public double turretAngle = 0.0;

    public static double turretPosAt0 = 0.44;
    public static double posChange90 = 0.34;


    // math stuff TODO calculate this
    public static double robotHeight = 14.0; //in, acctually shoudl be where the shooter is
    public static double g = 386.08858267717; //in per sec^2
    public static double goalHeight = 40.0; //doesnt change no matter alliance color
    public static double apexHeight = 60.0; //what the apex of the balls path is going to try to be

    public double goalPitch;
    public double goalVelocity;
    public double goalYaw;
    public double goalHoodPos;
    public double goalYawPos;
    public static double difference = 109.0;

    public static double minVelocity = 282.0 + difference; // in/sec, at 1
    public static double maxVelocity = 477.1 + difference; // in/sec, at 0.7
    public static double hoodPosMax = 0.35; //maximum position the servo can go to
    public static double hoodPosMin = 0.55; //min position the servo can go to
    public static double hoodAngleMax = 1.2217; //radian measure of hood at max pos
    public static double hoodAngleMin = 0.8726; //radian measure of hood at min pos
    public boolean isAutoAimOn;
    private final Robot robot;
    public static double velCoeff = 2.0;

    // turret stuff
    // 320 deg of servo rotation = 408 deg of turret rotation
    public static double YAW_GEAR_RATIO = 408.0 / 320.0;

    public ShooterSubsystem(RobotHardware hardware, Robot robot) {
        this.robot = robot;
        this.hardware = hardware;

        this.shooterPID.setTolerance(this.shooterVelocityTolerance);
        this.shooterPID.setTargetPosition(0.0);
        //currently doesnt control anything in this class, just for keeping track
        this.isAutoAimOn = true;


    }

    public void doAutoShoot(Pose2d goalPos){

        this.doAutoShoot(goalPos, Arc);
    }

    public void doAutoShoot(Pose2d goalPos, ShotType shotType){

        Pose botPosTemp = this.robot.follower.getPose();
        Log.d("shooter","bot pos 1"+ botPosTemp);
        Pose2d botPos = new Pose2d(botPosTemp.getX(), botPosTemp.getY(), botPosTemp.getHeading());
        this.isAutoAimOn = true;
        this.doMath(botPos, goalPos, shotType, apexHeight);
        //velocity is in inches/second, if this doesnt match the encoder we'll have to fix
        this.setSpeed(this.velToRPM(this.goalVelocity));
        //gets a setpos from the angle from our measured angles for max and min
        this.goalHoodPos = Algebra.mapRange(goalPitch, hoodAngleMin, hoodAngleMax, hoodPosMin, hoodPosMax);
        this.goalYawPos = this.findYawAngle(botPos, goalPos);


        this.setHoodPosition(this.goalHoodPos);

    }

    public void manualAim(double velocity, double pitch, double yaw){
        /** lets you set a velocity and angle manually*/
        this.isAutoAimOn = false;
        this.goalVelocity = velocity;
        this.goalPitch = pitch;
        this.setSpeed(this.velToRPM(this.goalVelocity));
        this.goalHoodPos = Algebra.mapRange(goalPitch, hoodAngleMin, hoodAngleMax, hoodPosMin, hoodPosMax);
        this.goalYaw = yaw;

        this.setHoodPosition(this.goalHoodPos);
        this.setTurretAngle(this.goalYawPos);
    }
    public void setVelocity(double velocity){
        manualAim(velocity, this.goalPitch, this.goalYaw);
    }
    public void setPitch(double pitch){
        manualAim(this.goalVelocity, pitch, this.goalYaw);
    }
    public void setYaw(double yaw){
        manualAim(this.goalVelocity, this.goalPitch, yaw);
    }

    public void doMath(Pose2d botPos, Pose2d goalPos, ShotType shotType, double arcHeight){
        Log.e("shooter", "running math...");
        /**
         * attempts to calculate a velocity and angle from the robot position and our apex height
         * i let you pass in a different value other than apexHeight above bc we might want to change that later
         * see discord for the way i got my formulas
         * the formulas return 2 sets of values, the first one tends to be a regular, arc shot
         * the second tends to be more of a backboard shot, this one the velocity usually is crazy high
         * so if u go for backboard its likley it wont find valid values unless ur careful with h */


        double h = arcHeight;
        int failCount = 0;
        double targetV;
        double targetT;

        for(int failcount=1;failcount<9;failcount++){
            //my formulas
            double horDist = Math.sqrt(Math.pow((botPos.x-goalPos.x),2) +
                    Math.pow((botPos.y-goalPos.y),2)); //simple pythagrean therom
            double verDist = goalHeight - robotHeight;
            double theta1 = Math.atan(((2*h)/horDist) *
                    (1 + Math.sqrt(1 - (verDist/h)))); //in radians, from math
            double theta2 = Math.atan(((2*h)/horDist) *
                    (1 - Math.sqrt(1 - (verDist/h))));
            double v1 = (Math.sqrt(2*g*h))/Math.sin(theta1);
            double v2 = (Math.sqrt(2*g*h))/Math.sin(theta2);
            //as said above, first values are more of an arc shot
            if (shotType == Arc){
                targetT = theta1;
                targetV = v1 * velCoeff + difference;
            }
            else {
                targetT = theta2;
                targetV = v2 * velCoeff + difference;
            }
            //detects if something about our target values are out of range.
            //tries 4 more times, trying to adjust h to get valid numbers
            //if it doesnt find any, it starts over with the other pair of values
            //if, after 8 times, it doesnt get anything, it throws an error
            if (targetV < minVelocity || targetV > maxVelocity || targetT < hoodAngleMin || targetT > hoodAngleMax){
                if (failCount == 4){
                    h = apexHeight;
                    if (shotType == Arc) shotType = Straight;
                    else shotType = Arc;
                }
                else if (failcount == 8){
                    if (targetV < minVelocity){
                        targetV = minVelocity;
                    }
                    if (targetV > maxVelocity){
                        targetV = maxVelocity;
                    }
                    if (targetT < hoodAngleMin){
                        targetT = hoodAngleMin;
                    }
                    if (targetT > hoodAngleMax){
                        targetT = hoodAngleMax;
                    }
                    this.goalVelocity = targetV;
                    this.goalPitch = targetT;

                    break;

                }

                if (targetV < minVelocity || targetT < hoodAngleMin){
                    h += 5;
                }
                else if (targetV > maxVelocity || targetT > hoodAngleMax){
                    h -= 5;
                }


            }
            else {
                this.goalVelocity = targetV;
                this.goalPitch = targetT;
                break;

            }

        }

        Log.e("shooter", "goal vel" + goalVelocity);
        Log.e("shooter", "goal pitch" +
                goalPitch);
    }
    private double findYawAngle(Pose2d botPos, Pose2d goalPos){
         double x = goalPos.x - botPos.x;
         double y = goalPos.y - botPos.x;
         double angle = Math.atan2(y,x);
         Log.d("shooter", "yaw angle" + angle);
         double absoluteGoalAngle = (angle-(0.5 * Math.PI))+0.5*Math.PI;
         Log.d("shooter", "absolute goal yaw angle" + absoluteGoalAngle);
         double botHeading = robot.follower.getHeading();
         Log.d("shooter", "bot heading" + botHeading);
         double angleGoalOffset = Angle.angleWrap(absoluteGoalAngle - botHeading);

         this.goalYaw = absoluteGoalAngle;

         double pos = Algebra.mapRangeNoClamp(angleGoalOffset, -0.5*Math.PI, 0.5*Math.PI,
                 turretPosAt0-posChange90, turretPosAt0+posChange90, -Math.PI, Math.PI);
         Log.d("shooter", "calc yaw pos" + pos);
         return pos;




    }
    public double getTargetAngle(){
        return this.goalPitch;
    }
    public double getTargetVelocity(){
        return this.goalVelocity;
    }
    public double getTargetHoodPos(){
        return this.goalHoodPos;
    }
    public double getTargetPitch(){return this.goalPitch;}



    public void setSpeed(double goal){
        this.shooterPID.setTargetPosition(goal);
    }

    public double getVelocity() {
        return this.hardware.shooterEncoder.getVelocity();
    }

    public double getVelocityRpm() {
        return ticksToRpm(getVelocity());
    }

    public double ticksToRpm(double ticksPerSec) {
        return ticksPerSec * 60.0 / TICKS_PER_REV;
    }

    public void updateShooter() {
        this.shooterSpeed = this.shooterPID.calculatePower(this.getVelocityRpm(),0);
    }

    public void setHoodPosition(double position){
        this.hoodPosition=position;
    }


    public void setTurretAngle(double angle) {
        this.turretAngle = Math.max(-Math.PI, Math.min(Math.PI, angle));
    }
    public double velToRPM(double velocity){
        return velocity * 6.469;

    }








    @Override
    public void periodic() {
        this.doAutoShoot(robot.goalPos);


        // shooter pitch
        hardware.shooterPitch.setPosition(Math.max(hoodPosMin, Math.min(hoodPosMax, this.hoodPosition)));

        // flywheel pids
        this.updateShooter();
//        hardware.shooterLeft.setPower(shooterSpeed);
//        hardware.shooterRight.setPower(shooterSpeed);

        // shooter rotation for turret
//        double servoYaw = this.turretAngle / YAW_GEAR_RATIO;

        hardware.turretYawLeft.setPosition(this.goalYawPos);
        hardware.turretYawRight.setPosition(this.goalYawPos);
    }
}
