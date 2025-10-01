package org.firstinspires.ftc.teamcode.robot.subsystems;

import android.util.Log;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.math.Algebra;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.math.controllers.PidfController;

public class ShooterSubsystem extends SubsystemBase {
    private final RobotHardware hardware;

    // TODO: tune velocity pid coefficients + tolerance
    public static PidfController.PidfCoefficients shooterPIDCoeffecients =
            new PidfController.PidfCoefficients(0.0, 0, 0.00, 0, 0);
    public static double shooterVelocityTolerance = 0.0;

    // the current pid + speed
    public final PidfController shooterPID = new PidfController(shooterPIDCoeffecients);
    public double shooterSpeed=0.0;

    // the current shooting angle
    public double hoodPosition= 0.0;
    public double turretAngle = 0.0;

    public static double turretPosAtZero = 0.0;
    public static double turretPosAt360 = 0.0;

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

    public static double minVelocity = 158.4; // in/sec, rn 9mph
    public static double maxVelocity = 299.2; // in/sec, rn 17mph
    public static double hoodPosMax = 1.0; //maximum position the servo can go to
    public static double hoodPosMin = 0.0; //min position the servo can go to
    public static double hoodAngleMax = 1.4; //radian measure of hood at max pos
    public static double hoodAngleMin = 0.78; //radian measure of hood at min pos
    public boolean isAutoAimOn;

    // turret stuff
    // 320 deg of servo rotation = 408 deg of turret rotation
    public static double YAW_GEAR_RATIO = 408.0 / 320.0;

    // TODO: tune these once we get bot. Min/max pos should be opposite extremes of
    //  turret yaw.
    public static double YAW_LEFT_MIN_POS = 0.0;
    public static double YAW_LEFT_MAX_POS = 1.0;
    public static double YAW_RIGHT_MIN_POS = 0.0;
    public static double YAW_RIGHT_MAX_POS = 1.0;

    public ShooterSubsystem(RobotHardware hardware) {

        this.hardware = hardware;

        this.shooterPID.setTolerance(this.shooterVelocityTolerance);
        this.shooterPID.setTargetPosition(0.0);
        //currently doesnt control anything in this class, just for keeping track
        this.isAutoAimOn = true;


    }

    public void doAutoShoot(Pose2d botPos, Pose2d goalPos){
        /** this is the function that should be called every loop
         rn i just have it supplying the bot pos, when we get localizer class i can impliment that instead
         i also take in the goal pos bc like red v blue, if theres a better way to do this lmk*/
        this.doAutoShoot(botPos, goalPos, "arc");
    }

    public void doAutoShoot(Pose2d botPos, Pose2d goalPos, String shotType){
        this.isAutoAimOn = true;
        this.doMath(botPos, goalPos, shotType, apexHeight);
        //velocity is in inches/second, if this doesnt match the encoder we'll have to fix
        this.setSpeed(this.goalVelocity);
        //gets a setpos from the angle from our measured angles for max and min
        this.goalHoodPos = Algebra.mapRange(goalPitch, hoodAngleMin, hoodAngleMax, hoodPosMin, hoodPosMax);
        this.goalYaw = this.findYawAngle(botPos, goalPos);
        this.goalYawPos = (turretPosAtZero-turretPosAt360)/(-2* Math.PI)*this.goalYaw;
        this.setHoodPosition(this.goalHoodPos);
        this.setTurretAngle(this.goalYawPos);
    }

    public void manualAim(double velocity, double pitch, double yaw){
        /** lets you set a velocity and angle manually*/
        this.isAutoAimOn = false;
        this.goalVelocity = velocity;
        this.goalPitch = pitch;
        this.setSpeed(goalVelocity);
        this.goalHoodPos = Algebra.mapRange(goalPitch, hoodAngleMin, hoodAngleMax, hoodPosMin, hoodPosMax);
        this.goalYaw = yaw;
        this.goalYawPos = (turretPosAtZero-turretPosAt360)/(-2* Math.PI)*this.goalYaw;
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

    private void doMath(Pose2d botPos, Pose2d goalPos, String shotType, double arcHeight){
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

        while (true){
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
            if (shotType == "arc"){
                targetT = theta1;
                targetV = v1;
            }
            else {
                targetT = theta2;
                targetV = v2;
            }
            //detects if something about our target values are out of range.
            //tries 4 more times, trying to adjust h to get valid numbers
            //if it doesnt find any, it starts over with the other pair of values
            //if, after 8 times, it doesnt get anything, it throws an error
            if (targetV < minVelocity || targetV > maxVelocity || targetT < hoodAngleMin || targetT > hoodAngleMax){
                if (failCount == 4){
                    h = apexHeight;
                    if (shotType == "arc") shotType = "backboard";
                    else shotType = "arc";
                }
                else if (failCount == 8){
                    Log.e("shooter", "no valid velocity and angle found with given location and h");
                    break;
                }
                if (targetV < minVelocity || targetT < hoodAngleMin){
                    h += 5;
                }
                else if (targetV > maxVelocity || targetT > hoodAngleMax){
                    h -= 5;
                }
                failCount += 1;
            }
            else {
                this.goalVelocity = targetV;
                this.goalPitch = targetT;
                break;

            }
        }
    }
    private double findYawAngle(Pose2d botPos, Pose2d goalPos){
         double x = Math.abs(botPos.x-goalPos.x);
         double y = Math.abs(botPos.y- goalPos.y);
         double angle = Math.tan(y/x);
         return angle;
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

    public double getShooterVelocity(){
        return this.hardware.shooterEncoder.getVelocity();
    }

    public void updateShooter() {
        this.shooterSpeed = this.shooterPID.calculatePower(this.getShooterVelocity(),0);
    }

    public void setHoodPosition(double position){
        this.hoodPosition=position;
    }


    public void setTurretAngle(double angle) {
        this.turretAngle = Math.max(-Math.PI, Math.min(Math.PI, angle));
    }







    @Override
    public void periodic() {
        // shooter pitch
        hardware.shooterPitch.setPosition(this.hoodPosition);

        // flywheel pids
        this.updateShooter();
        hardware.shooterLeft.setPower(shooterSpeed);
        hardware.shooterRight.setPower(shooterSpeed);

        // shooter rotation for turret
        double servoYaw = this.turretAngle / YAW_GEAR_RATIO;
        servoYaw = Math.max(-Math.PI, Math.min(Math.PI, servoYaw));
        hardware.turretYawLeft.setPosition(Algebra.mapRange(
                servoYaw,
                -Math.PI, Math.PI,
                YAW_LEFT_MIN_POS, YAW_LEFT_MAX_POS
        ));
        hardware.turretYawRight.setPosition(Algebra.mapRange(
                servoYaw,
                -Math.PI, Math.PI,
                YAW_RIGHT_MIN_POS, YAW_RIGHT_MAX_POS
        ));
    }
}
