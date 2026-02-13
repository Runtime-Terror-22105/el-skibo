package org.firstinspires.ftc.teamcode.robot.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.math.Algebra;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Coordinate;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.controllers.PidfController;
import org.firstinspires.ftc.teamcode.pedroPathing.FtcDashDrawing;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;
import org.firstinspires.ftc.teamcode.robot.subsystems.shooter.FlightTimeLookupTable;
import org.firstinspires.ftc.teamcode.robot.subsystems.shooter.GoalPosLookupTable;
import org.firstinspires.ftc.teamcode.robot.subsystems.shooter.ShooterLookupTable;
import org.firstinspires.ftc.teamcode.util.Profiler;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@Config
public class ShooterSubsystem extends SubsystemBase {
    public static boolean USE_SOTM = true;


    public static boolean debug = false;
    public static boolean telemetry = true;

    public static double TICKS_PER_REV = 28; // GoBilda yellowjacket encoder

    // TODO: tune velocity pid coefficients + tolerance
    public static PidfController.PidfCoefficients shooterPIDCoeffecients =
            new PidfController.PidfCoefficients(0.0005, 0, 0, 0.000196, 0);
    public final PidfController shooterPID = new PidfController(shooterPIDCoeffecients);
    public static double SHOOTER_VELOCITY_TOLERANCE = 0.0;


    public GoalPosLookupTable goalPosLookupTable;

    // what the shooter should be at
    public double goalPitch; //hood - rad
    public double goalVelocity; //flywheel - rpm
    public double goalTurretAngle; //turret - rad
    public double goalPitchPos; //hood - servo pos todo: remove, only have goalPitch

    // turret positions
    public static double turretOffset = 0.00; //turret manual offset- servo pos
    public static double turretPosAt180 = 0.48; //pos pointed directly towards the back
    public static double posChange90 = 0.35; //servo pos change that rotates turret 90 deg
    public static Coordinate turretToRobotCenterOffset = new Coordinate(-1.61417, 0);

    // in loops, how often to update the turret position servo when outside of the shooting zone
    public static double TURRET_UPDATE_FREQUENCY = 10;
    private double loopCount = 0;

    // No angle limit for turret, but we have servo positions limits
    public static double turretLowerBound = Math.toRadians(0);
    public static double turretUpperBound = Math.toRadians(360);
    public static double turretServoLowerBound = 0.0;
    public static double turretServoUpperBound = 1;

    // hood limits
    public static double hoodPosMax = 0.6; //maximum position the servo can go to
    public static double hoodPosMin = 0.05; //min position the servo can go to
    public static double hoodAngleMax = 0.919427826056; //radian measure of hood at max pos
    public static double hoodAngleMin = 0.632748891943; //radian measure of hood at min pos

    // vars for calculating shot (unused currently, todo: remove later)
    public static double robotHeight = 14.0; //in
    public static double g = 386.08858267717; //in per sec^2
    public static double goalHeight = 40.0; //doesnt change no matter alliance color
    public static double apexHeight = 60.0; //what the apex of the balls path is going to try to be

    private final RobotHardware hardware;
    private final Robot robot;

    // flags for autoshoot
    public boolean isAutoAimOn;
    public boolean isAutoVelOn;
    public boolean isAutoHoodOn;
    public boolean isAutoTurretOn;
    public boolean alwaysUpdateTurret = false;

    public static int rollingValLen = 5;
    public LinkedHashMap<Double, Double> velValues;
    public int ballsShot = 0;
    public ElapsedTime ballsShotTimer = new ElapsedTime();

    // If this is set, the robot will use this pose instead of the follower pose for auto-shoot
    // calculations. If null, the follower pose is used.
    //
    // This is useful during auto to avoid dynamically updating the shooter while the robot moves.
    public Pose autoShootPoseOverride = null;
    public Boolean sotmOverride = null;

    // flag used for lighting feedback for driver
    public boolean turretInDeadzone = false;

    public static class ShooterValues {
        public double velocity;
        public double rad;
        public ShooterValues(double v, double r){
            this.velocity = v;
            this.rad = r;
        }
    }

    public ShooterSubsystem(RobotHardware hardware, Robot robot) {
        this.robot = robot;
        this.hardware = hardware;

        this.shooterPID.setTolerance(SHOOTER_VELOCITY_TOLERANCE);
        this.shooterPID.setTargetPosition(0.0);

        this.goalPosLookupTable = new GoalPosLookupTable(this.robot);
        for (int i=0; i < rollingValLen; i++){
            velValues.put(0D, 0D);
        }



        //currently doesnt control anything in this class, just for keeping track
        this.isAutoAimOn = true;
        this.isAutoVelOn = true;
        this.isAutoHoodOn = true;
        this.isAutoTurretOn = true;
        this.turretInDeadzone = false;
    }

    public static double turretAngleToServoPos(double angleRad) {
        double unboundedServo = Algebra.mapRangeNoClamp(angleRad,
                Math.toRadians(90), Math.toRadians(270),
                turretPosAt180-posChange90, turretPosAt180+posChange90
        );
        return MathFunctions.clamp(unboundedServo, turretServoLowerBound, turretServoUpperBound);
    }

    public void doAutoShoot(Pose botPos, boolean useVelocityCompensation) {
        if (debug) Log.d("ShooterSubsystem", "Doing autoshoot!");
        this.isAutoAimOn = true;

        Pose2d goalPos = this.goalPosLookupTable.getForPose(botPos);
        double distToGoal = botPos.distanceFrom(goalPos.toPedro());
        FtcDashDrawing.drawDot(goalPos.toPedro(), "#000000");


        if (useVelocityCompensation) {
            double flightTime = FlightTimeLookupTable.get(distToGoal);
            Vector goalAdjAmt = robot.follower.getVelocity().times(flightTime);
            goalPos = Pose2d.minus(goalPos, goalAdjAmt);
            distToGoal = botPos.distanceFrom(goalPos.toPedro());

            if (debug) Log.d("ShooterSubsystem", "Adjusted goal pos for velocity: " + goalAdjAmt);
            FtcDashDrawing.drawDot(goalPos.toPedro(), "#0000FF");
        }

        //currently limited to 90 - 270 degrees, can be changed by changing the values in the map range below
        // also currently only updates when in the tape zone or every 10 loops to reduce wrtes
        if (isAutoTurretOn && (alwaysUpdateTurret || loopCount == 0 || robot.isInTapeZone())) {
            this.setTurretAngle(this.findYawAngle(botPos, goalPos));
        }


        ShooterValues math;
        math = ShooterLookupTable.get(distToGoal);
        //calcVelcoity - in/sec

        if (telemetry) Robot.debugTelemetry.addData("Calculated Velocity (in/sec)", math.velocity);


        //velocity is in inches/second, if this doesnt match the encoder we'll have to fix
        if (this.isAutoVelOn) {
            this.setSpeed(this.velToRPM(math.velocity)); // todo: add back
        }
        if (this.isAutoHoodOn && robot.robotState != RobotState.SHOOTING) {
            this.goalPitch = math.rad;
            this.goalPitchPos = Algebra.mapRange(math.rad, hoodAngleMin, hoodAngleMax, hoodPosMin, hoodPosMax);
        }
        if (telemetry) Robot.debugTelemetry.addData("Calculated Pitch (rad)", this.goalPitch);

        if (debug) Log.i("ShooterSubsystem", "Calculated flywheel velocity: " + this.getGoalVelocity() + " rpm");
        if (debug) Log.i("ShooterSubsystem", "Calculated hood pitch (rad)" + this.goalPitch);
    }

    public void setTurretAngle(double angleRad) {
        this.goalTurretAngle = Math.max(turretLowerBound, Math.min(turretUpperBound, angleRad));
    }

    public void calcHoodPod(Pose2d botPos, Pose2d goalPos, double arcHeight) {
        // note: arcHeight is usually set to the apexHeight variable, which is currently 60
        if (debug) Log.d("ShooterSubsystem", "running hood math");

        double h = arcHeight-robotHeight; // the delta y at the apex

        //my formulas
        double horDist = Math.sqrt(Math.pow((botPos.x-goalPos.x),2) +
                Math.pow((botPos.y-goalPos.y),2)); //simple pythagrean therom
        double verDist = goalHeight - robotHeight; // delta y at the goal
        double theta = Math.atan(((2*h)/horDist) *
                (1 + Math.sqrt(1 - (verDist/h)))); //in radians, from math
        this.goalPitch = theta;
        this.goalPitchPos = Algebra.mapRange(theta, hoodAngleMin, hoodAngleMax, hoodPosMin, hoodPosMax);
        if (debug) Log.d("ShooterSubsystem", "goal hood pos" + this.goalPitchPos);
    }

    /** lets you set a velocity and angle manually*/
    public void manualAimGoalPos(double velocity, double pitch, Pose2d goalPos) {
        this.isAutoAimOn = false;
        this.setSpeed(this.velToRPM(velocity));

        this.goalPitch = pitch;
        this.goalPitchPos = Algebra.mapRange(pitch, hoodAngleMin, hoodAngleMax, hoodPosMin, hoodPosMax);

        this.setTurretAngle(this.findYawAngle(this.robot.follower.getPose(), goalPos));
    }

    public void manualAim(double velocity, double pitch, double turretYaw) {
        Pose2d goalPos = this.goalPosLookupTable.get();

        this.isAutoAimOn = false;
        this.setSpeed(this.velToRPM(velocity));

        this.goalPitch = pitch;
        this.goalPitchPos = Algebra.mapRange(pitch, hoodAngleMin, hoodAngleMax, hoodPosMin, hoodPosMax);

        this.setTurretAngle(turretYaw);
    }


    public void manualAimAutoHood (double velocity, double turretYaw) {

        Pose botPosTemp = this.robot.follower.getPose();
        Pose2d botPos = new Pose2d(botPosTemp.getX(), botPosTemp.getY(), botPosTemp.getHeading());
        Pose2d goalPos = this.goalPosLookupTable.get();

        this.isAutoAimOn = false;
        this.setSpeed(this.velToRPM(velocity));

        if (this.isAutoHoodOn) {
            calcHoodPod(botPos, goalPos, apexHeight);
        }
        if (telemetry) Robot.debugTelemetry.addData("Calculated Pitch (rad)", this.goalPitch);

        if (debug) Log.d("ShooterSubsystem", "Calculated flywheel velocity: " + this.getGoalVelocity() + " rpm");
        if (debug) Log.d("ShooterSubsystem", "Calculated hood pitch (rad)" + this.goalPitch);


        this.setTurretAngle(turretYaw);
    }



    private double findYawAngle(Pose botPos, Pose2d goalPos){
        /** all in rad **/
        Pose robotVector = new Pose(turretToRobotCenterOffset.x, turretToRobotCenterOffset.y, 0)
                .rotate(botPos.getHeading(), false);
        Pose turretCenter = botPos.plus(robotVector);
        double dx = goalPos.x - turretCenter.getX();
        double dy = goalPos.y - turretCenter.getY();
        double angle = Math.atan2(dy, dx);

        double absoluteGoalAngle = angle;



        double botHeading = botPos.getHeading();
        if (telemetry) robot.telemetry.addData("follower heading (deg)",botHeading*180/Math.PI );


        // note: this is 0 to 360 instead of -180 to 180 for convenience below
        double angleTurret = Angle.normalize(absoluteGoalAngle - botHeading);
        if (debug) Log.d("ShooterSubsystem", "turret angle (deg): " + Math.toDegrees(angleTurret));
        if (debug) Log.d("ShooterSubsystem", "calculated servo pos: " + turretAngleToServoPos(angleTurret));

        Pose turretPose = new Pose(botPos.getX(), botPos.getY(), botPos.getHeading() + angleTurret);
        FtcDashDrawing.drawRobot(turretPose, "#FFFFFF");
        FtcDashDrawing.drawHeadingRay(turretPose, "FFFFFF");
        return angleTurret;
    }

    public void updateRollingVelValues(){
        velValues.remove(velValues.keySet().iterator().next());
        velValues.put(this.getVelocity(), this.getGoalVelocity());
    }

    public void checkShotBalls(){
        boolean valid = true;
        if (velValues.get(velValues.keySet().iterator().next()) - goalVelocity < 20){

            List<Double> differences = new ArrayList<>();
            for (Map.Entry<Double, Double> entry : velValues.entrySet()){
                 differences.add(Math.abs(entry.getKey())-entry.getValue());
            }
            double avg = 0;

            for (double d1: differences){
                for (double d2: differences){
                    if (Math.abs(d1-d2) > 10){
                        valid = false;
                    }
                }
                avg += d1;
            }
            avg = avg/rollingValLen;

            if (valid && Math.abs(goalVelocity-this.getVelocity()) *2 > avg){
                ballsShot +=1;
                Log.i("ShooterSubsystem.java", "Ball Shot!");
                ballsShotTimer.reset();
            }

        }
    }
    public int getBallsShot(){return ballsShot;}



    public double getGoalVelocity() {
        /* RPM */
        return this.goalVelocity;
    }

    public void setSpeed(Double goal) {
        /* updates goalVelocity */
        //goal should be in RPM
        if (debug) Log.d("ShooterSubsystem", "setSpeed (rpm): " + goal);
        if (goal != null)
            this.goalVelocity = goal;
    }

    public double getVelocity() {
        /*ENCODER VELOCITY IN TICKS*/
        return this.hardware.shooterEncoder.getVelocity();
    }

    public double getVelocityRpm() {
        /*ENCODER VELOCITY IN RPM*/
        return ticksToRpm(getVelocity());
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

    public double updateShooter() {
        if (telemetry) Robot.debugTelemetry.addData("Shooter RPM", this.getVelocityRpm());
        if (telemetry) Robot.debugTelemetry.addData("Shooter in/s", this.getVelocityRpm() / 6.469);
//        Robot.debugTelemetry.addData("Shooter left (mA)", this.hardware.shooterLeft.getCurrent(CurrentUnit.MILLIAMPS));
//        Robot.debugTelemetry.addData("Shooter right (mA)", this.hardware.shooterRight.getCurrent(CurrentUnit.MILLIAMPS));

        double ff = this.hardware.getVoltageScale() * getGoalVelocity();
        this.shooterPID.setTargetPosition(getGoalVelocity());
        return this.shooterPID.calculatePower(this.getVelocityRpm(), ff);
    }

    public void addTurretOffset(double change){
        this.turretOffset += change;
    }

    public double getGoalTurretYaw() {
        return this.goalTurretAngle;
    }

    @Override
    public void periodic() {
//
//        if (robot.color == Team.BLUE){
//            turretOffset = -0.04;
//        }
//        else{
//            turretOffset = 0.04;
//        }

        try (Profiler.Scope p = Profiler.enter("ShooterSubsystem")) {
            if (robot.robotState.isHang()) {
                hardware.shooterLeft.setPower(0);
                hardware.shooterRight.setPower(0);
                double turretYaw = turretAngleToServoPos(Math.toRadians(90.0)) + this.turretOffset;
                hardware.turretYawLeft.setPosition(turretYaw);
                hardware.turretYawRight.setPosition(turretYaw);
                return;
            }

            Profiler.push("ball shot logic");
            updateRollingVelValues();
            if (robot.robotState == RobotState.TRANSFER || robot.robotState == RobotState.SHOOTING){
                checkShotBalls();
            }
            if (ballsShotTimer.seconds() > 3 && ballsShot >0){
                ballsShot = 0;
            }
            Profiler.pop();



            Profiler.push("autoshoot");
            loopCount = (loopCount + 1) % TURRET_UPDATE_FREQUENCY;
            if (robot.goalPos != null && isAutoAimOn) {
                Pose robotPos;
                boolean useSotm;
//                if (this.autoShootPoseOverride != null) {
//                    robotPos = this.autoShootPoseOverride;
//                    useSotm = false;
//                } else {
                    robotPos = this.robot.follower.getPose();
                    useSotm = USE_SOTM;
//                    useSotm = sotmOverride != null ? sotmOverride : USE_SOTM;
//                }
                this.doAutoShoot(robotPos, useSotm);
            }
            else Log.e("ShooterSubsystem", "robot.goalPos is null! Skipping autoshoot...");
            Profiler.pop();

            // shooter pitch
            Profiler.push("pitch");
            hardware.shooterPitch.setPosition(this.goalPitchPos);
            Profiler.pop();

            // flywheel pids
            Profiler.push("flywheel");
            double shooterPower = this.updateShooter();
            Robot.debugTelemetry.addData("Shooter Power", shooterPower);
            hardware.shooterLeft.setPower(shooterPower);
            hardware.shooterRight.setPower(shooterPower);
            Profiler.pop();

            //turret
            Profiler.push("turret");
            double goalTurretPos = turretAngleToServoPos(this.goalTurretAngle) + this.turretOffset;
            this.turretInDeadzone = (goalTurretPos <= turretServoLowerBound) || (goalTurretPos >= turretServoUpperBound);
            hardware.turretYawLeft.setPosition(goalTurretPos);
            hardware.turretYawRight.setPosition(goalTurretPos);
            Profiler.pop();
        }
    }
}
