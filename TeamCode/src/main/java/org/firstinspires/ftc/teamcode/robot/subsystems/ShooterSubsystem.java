package org.firstinspires.ftc.teamcode.robot.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.math.Algebra;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.pedroPathing.FtcDashDrawing;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.math.controllers.PidfController;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;
import org.firstinspires.ftc.teamcode.robot.subsystems.shooter.GoalPosLookupTable;
import org.firstinspires.ftc.teamcode.robot.subsystems.shooter.ShooterLookupTable;
import org.firstinspires.ftc.teamcode.util.Profiler;

@Config
public class ShooterSubsystem extends SubsystemBase {

    private double loopCount = 0;

    // in loops, how often to update the turret position servo when outside of the shooting zone
    public static double TURRET_UPDATE_FREQUENCY = 10;

    private final RobotHardware hardware;

    public static boolean debug = false;
    public static boolean telemetry = true;
    public static boolean usingHardCodedShooterTable = false;
    public static double TICKS_PER_REV = 28; // GoBilda yellowjacket encoder

    // TODO: tune velocity pid coefficients + tolerance
    public static PidfController.PidfCoefficients shooterPIDCoeffecients =
            new PidfController.PidfCoefficients(0.0005, 0.0, 0.0, 0.000185, 0);
    public static double SHOOTER_VELOCITY_TOLERANCE = 0.0;

    // the current pid + speed
    public final PidfController shooterPID = new PidfController(shooterPIDCoeffecients);
    public double shooterPower = 0.0; //flywheel - motor power

    public static double turretPosAt180 = 0.49; //pos pointed directly towards the back
    public static double posChange90 = 0.38; //servo pos change that rotates turret 90 deg

    public double goalPitch; //hood - rad
    public double goalVelocity; //flywheel - rpm
    public double goalTurretAngle; //turret - rad

    public double goalPitchPos; //hood - servo pos

    public double turretOffset = 0.0; //turret manual offset- servo pos

    public GoalPosLookupTable goalPosLookupTable;

    public static double turretLowerBound = Math.PI/2; //currently 90 deg, var in rad
    public static double turretUpperBound = 3*Math.PI/2; //currently 270 deg, var in rad

    public static double hoodPosMax = 0.7; //maximum position the servo can go to
    public static double hoodPosMin = 0.15; //min position the servo can go to
    public static double hoodAngleMax = 0.919427826056; //radian measure of hood at max pos
    public static double hoodAngleMin = 0.632748891943; //radian measure of hood at min pos
    public static double robotHeight = 14.0; //in
    public static double g = 386.08858267717; //in per sec^2
    public static double goalHeight = 40.0; //doesnt change no matter alliance color
    public static double apexHeight = 60.0; //what the apex of the balls path is going to try to be

    public boolean isAutoAimOn;
    public boolean isAutoVelOn;
    public boolean isAutoHoodOn;
    public boolean isAutoTurretOn;
    private final Robot robot;
    public static class ShooterValues{
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

        //currently doesnt control anything in this class, just for keeping track
        this.isAutoAimOn = true;
        this.isAutoVelOn = true;
        this.isAutoHoodOn = true;
        this.isAutoTurretOn = true;
    }

    public static double turretAngleToServoPos(double angleRad) {
        return Algebra.mapRange(angleRad, turretLowerBound, turretUpperBound, turretPosAt180-posChange90, turretPosAt180+posChange90);
    }

    public void doAutoShoot(){
        if (debug) Log.d("ShooterSubsystem", "Doing autoshoot!");
        this.isAutoAimOn = true;

        Pose botPosTemp = this.robot.follower.getPose();
        Pose2d botPos = new Pose2d(botPosTemp.getX(), botPosTemp.getY(), botPosTemp.getHeading());
        Pose2d goalPos = this.goalPosLookupTable.get();
        FtcDashDrawing.drawDot(goalPos.toPedro(), "#000000");

        //currently limited to 90 - 270 degrees, can be changed by changing the values in the map range below
        // also currently only updates when in the tape zone or every 10 loops to reduce wrtes
        if (isAutoTurretOn && (loopCount == 0 || robot.isInTapeZone())) {
            this.setTurretAngle(this.findYawAngle(goalPos));
        }


        ShooterValues math;
//        if(usingHardCodedShooterTable)
//        {
//            calcVelocity = HardCodedLookup.get(botPos.toPedro().distanceFrom(goalPos.toPedro()));
//        }
//        else{
//            math = ShooterLookupTable.get(botPos.toPedro().distanceFrom(goalPos.toPedro()));
//        }
        math = ShooterLookupTable.get(botPos.toPedro().distanceFrom(goalPos.toPedro()));
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


    public void doAutoShootWithVelocityCompensation() {
        Log.i("shooter", "Doing autoshoot with velocity compensation!");
        this.isAutoAimOn = true;


        Pose botPosTemp = this.robot.follower.getPose();
        Pose2d botPos = new Pose2d(botPosTemp.getX(), botPosTemp.getY(), botPosTemp.getHeading());
        Pose2d goalPos = this.goalPosLookupTable.get();
        FtcDashDrawing.drawDot(goalPos.toPedro(), "#000000");


        // Get robot velocity vector from follower
        Vector robotVelocity = this.robot.follower.getVelocity();
        double robotVelMagnitude = robotVelocity.getMagnitude(); // in/s

        Log.d("robot vel mag", String.valueOf(robotVelMagnitude));

        double distanceToGoal = Math.sqrt(Math.pow(botPos.x - goalPos.x, 2) + Math.pow(botPos.y - goalPos.y, 2));
        double verDist = goalHeight - robotHeight;

        double angleToGoal1=Math.atan2(goalPos.y-botPos.y,goalPos.x-botPos.x);
        double robotHeading=botPos.heading;

        robot.telemetry.addData("Angle", Math.toDegrees(angleToGoal1));
        robot.telemetry.addData("heading current", Math.toDegrees(robotHeading));
        robot.telemetry.addData("distance to goal",distanceToGoal);
        robot.telemetry.addData("magnitude",robotVelMagnitude);
        robot.telemetry.update();


        ShooterValues math = ShooterLookupTable.get(botPos.toPedro().distanceFrom(goalPos.toPedro()));
        double finalVelocity = math.velocity;
        double finalLaunchAngle =math.rad;
        double turretOffsetAngle = 0.0;

        double robotVelAngle = robotVelocity.getTheta();
        double theta = Angle.normalize(robotVelAngle - angleToGoal1);

        double Vrr = -Math.cos(theta) * robotVelMagnitude;

        double Vrt = Math.sin(theta) * robotVelMagnitude;

        double timeToGoal = distanceToGoal / (math.velocity * Math.cos(math.rad));

        double VxCompensated = (distanceToGoal / timeToGoal) + Vrr;

        double VxNew = Math.sqrt(VxCompensated * VxCompensated + Vrt * Vrt);
        double Vy = math.velocity * Math.sin(math.rad);
        double newLaunchAngle = Math.atan2(Vy, VxNew);
        newLaunchAngle = Math.max(hoodAngleMin, Math.min(hoodAngleMax, newLaunchAngle));

        double newDistanceX = VxNew * timeToGoal;
        double newVelocitySquared = (g * newDistanceX * newDistanceX) /
                (2 * Math.cos(newLaunchAngle) * Math.cos(newLaunchAngle) *
                        (newDistanceX * Math.tan(newLaunchAngle) - verDist));
        double newVelocity = Math.sqrt(Math.max(0, newVelocitySquared));

        turretOffsetAngle = Math.tan(Vrt/VxCompensated);

        finalVelocity = newVelocity;
        finalLaunchAngle = newLaunchAngle;

        double ang = this.findYawAngle(goalPos);
        ang += turretOffsetAngle;
        ang = Math.max(turretLowerBound, Math.min(turretUpperBound, ang));
        this.setTurretAngle(ang);

        if (this.isAutoVelOn) {
            this.setSpeed(this.velToRPM(finalVelocity));
        }



        if (this.isAutoHoodOn && robot.robotState != RobotState.SHOOTING) {
            this.goalPitch = finalLaunchAngle;
            this.goalPitchPos = Algebra.mapRange(finalLaunchAngle, hoodAngleMin, hoodAngleMax, hoodPosMin, hoodPosMax);
        }

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

        this.setTurretAngle(this.findYawAngle(goalPos));
    }

    public void manualAim(double velocity, double pitch, double turretYaw) {
        Pose2d goalPos = this.goalPosLookupTable.get();

        this.isAutoAimOn = false;
        this.setSpeed(this.velToRPM(velocity));

        this.goalPitch = pitch;
        this.goalPitchPos = Algebra.mapRange(pitch, hoodAngleMin, hoodAngleMax, hoodPosMin, hoodPosMax);

        this.setTurretAngle(this.findYawAngle(goalPos));
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



    private double findYawAngle(Pose2d goalPos){
         /** all in rad **/
         double x = goalPos.x - robot.follower.getPose().getX();
         double y = goalPos.y - robot.follower.getPose().getY();
         double angle = Math.atan2(y,x);

         double absoluteGoalAngle = angle;

         double botHeading = robot.follower.getHeading();

        if (telemetry) robot.telemetry.addData("follower heading (deg)",botHeading*180/Math.PI );


        // note: this is 0 to 360 instead of -180 to 180 for convenience below
         double angleTurret = Angle.normalize(absoluteGoalAngle - botHeading);
         return angleTurret;
    }


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

    public void updateShooter() {
        if (telemetry) Robot.debugTelemetry.addData("Shooter RPM", this.getVelocityRpm());
        if (telemetry) Robot.debugTelemetry.addData("Shooter in/s", this.getVelocityRpm() / 6.469);
//        Robot.debugTelemetry.addData("Shooter left (mA)", this.hardware.shooterLeft.getCurrent(CurrentUnit.MILLIAMPS));
//        Robot.debugTelemetry.addData("Shooter right (mA)", this.hardware.shooterRight.getCurrent(CurrentUnit.MILLIAMPS));

        double ff = this.hardware.getVoltageScale() * getGoalVelocity();
        this.shooterPID.setTargetPosition(getGoalVelocity());
        this.shooterPower = this.shooterPID.calculatePower(this.getVelocityRpm(), ff);
    }

    public void addTurretOffset(double change){
        this.turretOffset += change;
    }

    public double getGoalTurretYaw() {
        return this.goalTurretAngle;
    }

    @Override
    public void periodic() {
        try (Profiler.Scope p = Profiler.enter("ShooterSubsystem")) {
            if (robot.hang.isPtoEngaged()) {
                hardware.shooterLeft.setPower(0);
                hardware.shooterRight.setPower(0);
                return;
            }



            Profiler.push("autoshoot");
            loopCount = (loopCount + 1) % TURRET_UPDATE_FREQUENCY;
            if (robot.goalPos != null && isAutoAimOn) {
                if (Robot.USE_SOTM) this.doAutoShootWithVelocityCompensation();
                else this.doAutoShoot();
            }
            else Log.e("ShooterSubsystem", "robot.goalPos is null! Skipping autoshoot...");
            Profiler.pop();

            // shooter pitch
            Profiler.push("pitch");
            hardware.shooterPitch.setPosition(this.goalPitchPos);
            Profiler.pop();

            // flywheel pids
            Profiler.push("flywheel");
            this.updateShooter();
            Robot.debugTelemetry.addData("Shooter Power", shooterPower);
            hardware.shooterLeft.setPower(shooterPower);
            hardware.shooterRight.setPower(shooterPower);
            Profiler.pop();

            //turret
            Profiler.push("turret");
            double goalTurretPos = turretAngleToServoPos(this.goalTurretAngle);
            hardware.turretYawLeft.setPosition(goalTurretPos + this.turretOffset);
            hardware.turretYawRight.setPosition(goalTurretPos + this.turretOffset);
            Profiler.pop();
        }
    }
}
