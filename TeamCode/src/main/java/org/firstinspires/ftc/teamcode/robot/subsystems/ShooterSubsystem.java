package org.firstinspires.ftc.teamcode.robot.subsystems;

import android.util.Log;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.math.Pose2d;
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

    public static double robotHeight = 14.0; //in, acctually shoudl be where the shooter is
    public static double g = 386.08858267717; //in per sec^2
    public static double goalHeight = 40.0; //doesnt change no matter alliance color
    public static double apexHeight = 60.0; //what the apex of the balls path is going to try to be
    public double goalAngle;
    public double goalVelocity;
    public static double goalHoodPos;
    public static double minVelocity = 158.4; // in/sec, rn 9mph
    public static double maxVelocity = 299.2; // in/sec, rn 17mph
    public static double hoodPosMax = 1.0; //maximum position the servo can go to
    public static double hoodPosMin = 0.0; //min position the servo can go to
    public static double hoodAngleMax = 1.4; //radian measure of hood at max pos
    public static double hoodAngleMin = 0.17; //radian measure of hood at min pos


    public boolean isAutoAimOn;
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
        this.setSpeed(goalVelocity);
        //gets a setpos from the angle from our measured angles for max and min
        goalHoodPos = ((hoodAngleMax - hoodAngleMin)/(hoodPosMax-hoodPosMin)) * goalAngle;
        setHoodPosition(goalHoodPos);
    }
    public void manualAim(double velocity, double angle){
        /** lets you set a velocity and angle manually*/
        this.isAutoAimOn = false;
        goalVelocity = velocity;
        goalAngle = angle;
        this.setSpeed(goalVelocity);
        goalHoodPos = ((hoodAngleMax - hoodAngleMin)/(hoodPosMax-hoodPosMin)) * goalAngle;
        setHoodPosition(goalHoodPos);
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
                this.goalAngle = targetT;
                break;

            }
        }



    }
    public double getTargetAngle(){
        return goalAngle;
    }
    public double getTargetVelocity(){
        return goalVelocity;
    }
    public double getTargetHoodPos(){
        return goalHoodPos;
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
