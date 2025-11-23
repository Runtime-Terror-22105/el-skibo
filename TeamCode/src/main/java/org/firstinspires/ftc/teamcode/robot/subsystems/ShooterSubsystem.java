package org.firstinspires.ftc.teamcode.robot.subsystems;

import static org.firstinspires.ftc.teamcode.robot.subsystems.ShotType.Arc;
import static org.firstinspires.ftc.teamcode.robot.subsystems.ShotType.Straight;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.math.Algebra;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.math.controllers.PidfController;
import org.firstinspires.ftc.teamcode.robot.subsystems.shooter.ShooterLookupTable;

@Config
public class ShooterSubsystem extends SubsystemBase {
    private final RobotHardware hardware;

    public static double TICKS_PER_REV = 28; // GoBilda yellowjacket encoder

    // TODO: tune velocity pid coefficients + tolerance
    public static PidfController.PidfCoefficients shooterPIDCoeffecients =
            new PidfController.PidfCoefficients(0.0001, 0.000115, 0.00, 0, 0);
    public static double SHOOTER_VELOCITY_TOLERANCE = 0.0;

    // the current pid + speed
    public final PidfController shooterPID = new PidfController(shooterPIDCoeffecients);
    public double shooterSpeed=0.0;

    // the current shooting angle
    public double turretAngle = 0.0;

    public static double turretPosAt180 = 0.54;
    public static double posChange90 = 0.38;


    // math stuff TODO calculate this
    public static double robotHeight = 14.0; //in, acctually shoudl be where the shooter is
    public static double g = 386.08858267717; //in per sec^2
    public static double goalHeight = 40.0; //doesnt change no matter alliance color
    public static double apexHeight = 60.0; //what the apex of the balls path is going to try to be

    private double goalPitch;
    private double goalVelocity;
    public double goalYawPos;
    public static double difference = 109.0;

    public static double minVelocity = 282.0 + difference; // in/sec, at 1
    public static double maxVelocity = 477.1 + difference; // in/sec, at 0.7
    public static double hoodPosMax = 0.35; //maximum position the servo can go to
    public static double hoodPosMin = 0.55; //min position the servo can go to
    public static double hoodAngleMax = 1.2217; //radian measure of hood at max pos
    public static double hoodAngleMin = 0.8726; //radian measure of hood at min pos

    public boolean isAutoAimOn;
    public boolean isAutoVelOn;
    public boolean isAutoHoodOn;
    private final Robot robot;
    public static double velCoeff = 1.8;

    // turret stuff
    // 320 deg of servo rotation = 408 deg of turret rotation
    public static double YAW_GEAR_RATIO = 408.0 / 320.0;

    public ShooterSubsystem(RobotHardware hardware, Robot robot) {
        this.robot = robot;
        this.hardware = hardware;

        this.shooterPID.setTolerance(SHOOTER_VELOCITY_TOLERANCE);
        this.shooterPID.setTargetPosition(0.0);
        //currently doesnt control anything in this class, just for keeping track
        this.isAutoAimOn = true;
        this.isAutoVelOn = true;
        this.isAutoHoodOn = true;

    }

    public void doAutoShoot(Pose2d goalPos){
        this.doAutoShoot(goalPos, Arc);
    }

    public void doAutoShoot(Pose2d goalPos, ShotType shotType){
        Log.i("shooter", "Doing autoshoot!");
        Pose botPosTemp = this.robot.follower.getPose();
        Pose2d botPos = new Pose2d(botPosTemp.getX(), botPosTemp.getY(), botPosTemp.getHeading());
        this.isAutoAimOn = true;

        this.goalYawPos = this.findYawAngle(goalPos);

//        ShooterValues math = this.doMath(botPos, goalPos, shotType, apexHeight);
        ShooterValues math = ShooterLookupTable.get(botPos.toPedro().distanceFrom(goalPos.toPedro()));
        if (math.flywheelVelocity == null || math.hoodPitch == null) {
            Log.e("shooter", "failed to do math!");
            return;
        }
        Robot.debugTelemetry.addData("Calculated Velocity", math.flywheelVelocity);
        Robot.debugTelemetry.addData("Calculated Pitch", math.hoodPitch);

        //velocity is in inches/second, if this doesnt match the encoder we'll have to fix
        if (this.isAutoVelOn) {
            this.setSpeed(this.velToRPM(math.flywheelVelocity)); // todo: add back
        }
        if (this.isAutoHoodOn) {
            //gets a setpos from the angle from our measured angles for max and min
            this.setGoalPitch(Algebra.mapRange(math.hoodPitch, hoodAngleMin, hoodAngleMax, hoodPosMin, hoodPosMax));
        }

        Log.i("shooter", "Calculated flywheel velocity: " + this.getGoalVelocity() + " rpm");
        Log.i("shooter", "Calculated hood pitch " + this.getGoalPitch());
    }

    /** lets you set a velocity and angle manually*/
    public void manualAim(double velocity, double pitch, double yaw) {
        this.isAutoAimOn = false;
        this.setSpeed(this.velToRPM(velocity));
        this.setGoalPitch(Algebra.mapRange(pitch, hoodAngleMin, hoodAngleMax, hoodPosMin, hoodPosMax));
        this.goalYawPos = yaw;

        this.setTurretAngle(this.goalYawPos);
    }
//
//    public void setVelocity(double velocity){
//        manualAim(velocity, this.goalPitch, this.goalYaw);
//    }
//    public void setPitch(double pitch){
//        manualAim(this.goalVelocity, pitch, this.goalYaw);
//    }
//    public void setYaw(double yaw){
//        manualAim(this.goalVelocity, this.goalPitch, yaw);
//    }

    public static class ShooterValues {
        public Double flywheelVelocity;
        public Double hoodPitch;

        public ShooterValues(Double flywheelVelocity, Double hoodPitch) {
            this.flywheelVelocity = flywheelVelocity;
            this.hoodPitch = hoodPitch;
        }
    }

    /**
     * attempts to calculate a velocity and angle from the robot position and our apex height
     * i let you pass in a different value other than apexHeight above bc we might want to change that later
     * see discord for the way i got my formulas
     * the formulas return 2 sets of values, the first one tends to be a regular, arc shot
     * the second tends to be more of a backboard shot, this one the velocity usually is crazy high
     * so if u go for backboard its likley it wont find valid values unless ur careful with h */
    public ShooterValues doMath(Pose2d botPos, Pose2d goalPos, ShotType shotType, double arcHeight){
        Log.e("shooter", "running math...");


        double h = arcHeight;
        double targetV;
        double targetT;

        for(int failCount=1;failCount<9;failCount++){
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
                else if (failCount == 8){
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
//                    this.goalVelocity = targetV;
//                    setSpeed(targetV); TODO: add back
//                    this.goalPitch = targetT;
                    return new ShooterValues(targetV, targetT);
                }

                if (targetV < minVelocity || targetT < hoodAngleMin){
                    h += 5;
                }
                else if (targetV > maxVelocity || targetT > hoodAngleMax){
                    h -= 5;
                }


            }
            else {
                return new ShooterValues(targetV, targetT);
            }
        }

        Log.e("shooter", "goal vel" + getGoalVelocity());
        Log.e("shooter", "goal pitch" + getGoalPitch());
        return new ShooterValues(null, null);
    }

    private double findYawAngle(Pose2d goalPos){
         double x = goalPos.x - robot.follower.getPose().getX();
         double y = goalPos.y - robot.follower.getPose().getY();
         double angle = Math.atan2(y,x);



         double absoluteGoalAngle = angle;

         double botHeading = robot.follower.getHeading();

        robot.telemetry.addData("follower",botHeading*180/Math.PI );


        // note: this is 0 to 360 instead of -180 to 180 for convenience below
         double angleTurret = Angle.normalize(absoluteGoalAngle - botHeading);
         return angleTurret;

//         // todo: this is currently limited to 90 to 270 degrees
//         double servopos = Algebra.mapRange(angleTurret, Math.PI/2, 3*Math.PI/2, turretPosAt180-posChange90, turretPosAt180+posChange90);
//
//        if ((angleTurret < turretPosAt180-posChange90 || angleTurret > turretPosAt180+posChange90) &&
//                robot.robotState == RobotState.READY_TO_SHOOT) {
//            robot.robotState = RobotState.NOT_READY;
//        }
//
//        if ((angleTurret > turretPosAt180-posChange90 && angleTurret < turretPosAt180+posChange90)
//                && robot.robotState == RobotState.NOT_READY) { // if we're in NOT_READY but are in range, we can set it back to ready
//            robot.robotState = RobotState.READY_TO_SHOOT;
//        }

//        robot.telemetry.addData("Goal Angle",Math.toDegrees(absoluteGoalAngle));
//        robot.telemetry.addData("X diff",x);
//        robot.telemetry.addData("Y diff",y);
//        robot.telemetry.addData("follower actual",robot.follower.getHeading());
//        robot.telemetry.addData("Angle of turret", Math.toDegrees(angleTurret));
//        robot.telemetry.addData("Servopos", Math.toDegrees(servopos));
//        robot.telemetry.addData("Pos of turret", pos);
//
//         return servopos;
    }

    public double getGoalVelocity() {
        return this.goalVelocity;
    }

    public void setSpeed(Double goal) {
        Log.d("shooter", "setSpeed: "+goal);
        if (goal != null)
            this.goalVelocity = goal;
    }

    public double getVelocity() {
        return this.hardware.shooterEncoder.getVelocity();
    }

    public double getVelocityRpm() {
        return ticksToRpm(getVelocity());
    }

    public double getGoalPitch() {
        return this.goalPitch;
    }

    public void setGoalPitch(double goalPitch) {
        this.goalPitch = goalPitch;
    }

    public double ticksToRpm(double ticksPerSec) {
        return ticksPerSec * 60.0 / TICKS_PER_REV;
    }

    /**
     * Convert inches/sec to rpm
     * @param velocity Velocity in in/sec
     * @return Velocity in RPM
     */
    public static double velToRPM(double velocity){
        return velocity * 6.469;
    }

    public void updateShooter() {
        Robot.debugTelemetry.addData("Shooter RPM", this.getVelocityRpm());
        Robot.debugTelemetry.addData("Shooter in/s", this.getVelocityRpm() / 6.469);
        Robot.debugTelemetry.addData("Shooter left (mA)", this.hardware.shooterLeft.getCurrent(CurrentUnit.MILLIAMPS));
        Robot.debugTelemetry.addData("Shooter right (mA)", this.hardware.shooterRight.getCurrent(CurrentUnit.MILLIAMPS));
        this.shooterPID.setTargetPosition(getGoalVelocity());
        this.shooterSpeed = this.shooterPID.calculatePower(this.getVelocityRpm(),0);
    }

    public void setTurretAngle(double angle) {
        this.turretAngle = Math.max(-Math.PI, Math.min(Math.PI, angle));
    }

    @Override
    public void periodic() {
        if (robot.goalPos != null && isAutoAimOn) this.doAutoShoot(robot.goalPos);
        else Log.e("ShooterSubsystem", "robot.goalPos is null! Skipping autoshoot...");


        // shooter pitch
        hardware.shooterPitch.setPosition(this.getGoalPitch());

        // flywheel pids
        this.updateShooter();
        Robot.debugTelemetry.addData("Shooter Power", shooterSpeed);
        hardware.shooterLeft.setPower(shooterSpeed);
        hardware.shooterRight.setPower(shooterSpeed);

        // shooter rotation for turret
//        double servoYaw = this.turretAngle / YAW_GEAR_RATIO;

        // todo: this is currently limited to 90 to 270 degrees
        double servoPosTurret = Algebra.mapRange(this.goalYawPos, Math.PI/2, 3*Math.PI/2, turretPosAt180-posChange90, turretPosAt180+posChange90);
        hardware.turretYawLeft.setPosition(servoPosTurret);
        hardware.turretYawRight.setPosition(servoPosTurret);
    }
}
