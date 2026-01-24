package org.firstinspires.ftc.teamcode.robot.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.seattlesolvers.solverslib.command.SubsystemBase;

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
    public static boolean DEBUG = false;
    public static boolean USE_TELEMETRY = true;

    public static double ENCODER_TICKS_PER_REV = 28; // GoBilda yellowjacket encoder
    private static final double IN_PER_SEC_TO_RPM = 6.469;

    public static double turretPosAt180 = 0.49; // pos pointed directly towards the back
    public static double posChange90 = 0.38; // servo pos change that rotates turret 90 deg
    public static double turretLowerBound = Math.PI/2; // currently 90 deg, var in rad
    public static double turretUpperBound = 3*Math.PI/2; // currently 270 deg, var in rad

    public static double TURRET_UPDATE_FREQUENCY = 10; // in loops, how often to update the turret position servo outside shooting zone

    public static double hoodPosMax = 0.7; // maximum position the servo can go to
    public static double hoodPosMin = 0.15; // min position the servo can go to
    public static double hoodAngleMax = 0.919427826056; // radian measure of hood at max pos
    public static double hoodAngleMin = 0.632748891943; // radian measure of hood at min pos
    public static double robotHeight = 14.0; // in

    public static double g = 386.08858267717; //in per sec^2
    public static double goalHeight = 40.0; //doesnt change no matter alliance color
    public static double apexHeight = 60.0; //what the apex of the balls path is going to try to be


    private final RobotHardware hardware;
    private final Robot robot;

    public final PidfController shooterPID = new PidfController(shooterPIDCoeffecients);
    public static PidfController.PidfCoefficients shooterPIDCoeffecients =
            new PidfController.PidfCoefficients(0.0005, 0.0, 0.0, 0.000185, 0);
    public static double SHOOTER_VELOCITY_TOLERANCE = 0.0;

    private double loopCount = 0;

    private double goalVelocity; // flywheel - rpm
    private double hoodPitch; // hood - rad
    private double turretAngle; // turret - rad

    public GoalPosLookupTable goalPosLookupTable;

    // params to control auto shooting
    public boolean isAutoAimOn;
    public boolean isAutoVelOn;
    public boolean isAutoHoodOn;
    public boolean isAutoTurretOn;

    public static class ShooterValues {
        public double velocity;
        public double rad;

        public ShooterValues(double v, double r){
            this.velocity = v;
            this.rad = r;
        }
    }

    public enum VelocityUnit {
        RPM,
        TICKS_PER_SEC,
        INCHES_PER_SEC
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

    // conversion functions

    /**
     * Convert turret angle in radians to servo position
     * @param angleRad Angle in radians
     * @return Servo position (0 to 1)
     */
    public static double turretAngleToServoPos(double angleRad) {
        return Algebra.mapRange(angleRad, turretLowerBound, turretUpperBound, turretPosAt180-posChange90, turretPosAt180+posChange90);
    }

    /**
     * Convert hood angle in radians to servo position
     * @param angleRad Angle in radians
     * @return Servo position (0 to 1)
     */
    public static double hoodAngleToServoPos(double angleRad) {
        return Algebra.mapRange(angleRad, hoodAngleMin, hoodAngleMax, hoodPosMin, hoodPosMax);
    }

    /**
     * Get hood angle in radians, controls the angle of the shot
     * @return Hood angle in radians
     */
    public double getHoodAngle() {
        return this.hoodPitch;
    }

    /**
     * Get hood servo position (0 to 1)
     * @return Hood servo position (0 to 1)
     */
    public double getHoodServoPos() {
        return hoodAngleToServoPos(this.getHoodAngle());
    }

    /**
     * Set hood angle in radians, controls the angle of the shot
     * @param angleRad Hood angle in radians
     */
    public void setHoodAngle(double angleRad) {
        this.hoodPitch = Math.max(hoodAngleMin, Math.min(hoodAngleMax, angleRad));
    }

    /**
     * Get goal flywheel velocity in RPM
     * @return
     */
    public double getGoalVelocity() {
        return this.goalVelocity;
    }

    public double getGoalVelocity(VelocityUnit unit) throws IllegalArgumentException {
        switch (unit) {
            case TICKS_PER_SEC:
                return this.goalVelocity * ENCODER_TICKS_PER_REV / 60.0;
            case RPM:
                return this.goalVelocity;
            case INCHES_PER_SEC:
                return rpmToInPerSec(this.goalVelocity);
            default:
                throw new IllegalArgumentException("Unsupported velocity unit: " + unit);
        }
    }

    /**
     * Set goal flywheel velocity in RPM
     * Mostly use {@link #setSpeed(Double goal)} instead for better error handling
     * @param rpm Goal velocity in RPM
     */
    public void setGoalVelocity(double rpm) {
        this.goalVelocity = rpm;
    }

    /**
     * Get turret angle in radians
     * @return Turret angle in radians
     */
    public double getTurretAngle() {
        return this.turretAngle;
    }

    public double getTurretServoPos() {
        return turretAngleToServoPos(this.getTurretAngle());
    }

    /**
     * Set turret angle in radians
     * @param angleRad Turret angle in radians
     */
    public void setTurretAngle(double angleRad) {
        this.turretAngle = Math.max(turretLowerBound, Math.min(turretUpperBound, angleRad));
    }

    /**
     * Set goal flywheel velocity in RPM
     * @param goal Goal velocity in RPM
     */
    public void setSpeed(Double goal) {
        /* updates goalVelocity */
        //goal should be in RPM
        if (DEBUG) Log.d("ShooterSubsystem", "setSpeed (rpm): " + goal);
        if (goal != null)
            this.goalVelocity = goal;
    }

    /**
     * Get current flywheel velocity in RPM
     * <p>Note: this is the current velocity, not the goal velocity.
     * For that, use {@link #getGoalVelocity()}</p>
     * @return Current flywheel velocity in ticks/sec
     */
    public double getVelocity() {
        return this.getVelocity(VelocityUnit.RPM);
    }

    /**
     * Get current flywheel velocity in a specified unit
     * <p>Note: this is the current velocity, not the goal velocity.
     * For that, use {@link #getGoalVelocity()}</p>
     * @param unit Desired velocity unit
     * @return Current flywheel velocity in specified unit
     */
    public double getVelocity(VelocityUnit unit) throws IllegalArgumentException {
        double velocityTicks = this.hardware.shooterEncoder.getVelocity();
        switch (unit) {
            case TICKS_PER_SEC:
                return velocityTicks;
            case RPM:
                return ticksToRpm(velocityTicks);
            case INCHES_PER_SEC:
                return rpmToInPerSec(ticksToRpm(velocityTicks));
            default:
                throw new IllegalArgumentException("Unsupported velocity unit: " + unit);
        }
    }

    /**
     * Convert ticks/sec to rpm
     * @param ticksPerSec Velocity in ticks/sec
     * @return Velocity in RPM
     */
    private static double ticksToRpm(double ticksPerSec) {
        return ticksPerSec * 60.0 / ENCODER_TICKS_PER_REV;
    }

    /**
     * Convert inches/sec to rpm
     * @param velocity Velocity in in/sec
     * @return Velocity in RPM
     */
    private static double inPerSecToRPM(double velocity){
        return velocity * IN_PER_SEC_TO_RPM;
    }

    /**
     * Convert rpm to inches/sec
     * @param velocity Velocity in in/sec
     * @return Velocity in RPM
     */
    private static double rpmToInPerSec(double velocity) {
        return velocity / IN_PER_SEC_TO_RPM;
    }

    public void doAutoShoot(){
        if (DEBUG) Log.d("ShooterSubsystem", "Doing autoshoot!");
        this.isAutoAimOn = true;

        Pose botPosTemp = this.robot.follower.getPose();
        Pose2d botPos = new Pose2d(botPosTemp.getX(), botPosTemp.getY(), botPosTemp.getHeading());
        Pose2d goalPos = this.goalPosLookupTable.get();
        FtcDashDrawing.drawDot(goalPos.toPedro(), "#000000");

        //currently limited to 90 - 270 degrees, can be changed by changing the values in the map range below
        // also currently only updates when in the tape zone or every 10 loops to reduce wrtes
        if (isAutoTurretOn && (loopCount == 0 || robot.isInTapeZone())) {
            this.setTurretAngle(this.calculateTurretAngle(goalPos));
        }


        ShooterValues math = ShooterLookupTable.get(botPos.toPedro().distanceFrom(goalPos.toPedro()));
        //calcVelcoity - in/sec

        if (USE_TELEMETRY) Robot.debugTelemetry.addData("Calculated Velocity (in/sec)", math.velocity);


        //velocity is in inches/second, if this doesnt match the encoder we'll have to fix
        if (this.isAutoVelOn) {
            this.setSpeed(inPerSecToRPM(math.velocity));
        }
        if (this.isAutoHoodOn && robot.robotState != RobotState.SHOOTING) {
            this.setHoodAngle(math.rad);
        }
        if (USE_TELEMETRY) Robot.debugTelemetry.addData("Calculated Pitch (rad)", this.getHoodAngle());

        if (DEBUG) Log.i("ShooterSubsystem", "Calculated flywheel velocity: " + this.getGoalVelocity() + " rpm");
        if (DEBUG) Log.i("ShooterSubsystem", "Calculated hood pitch (rad)" + this.getHoodAngle());
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

        if (USE_TELEMETRY) robot.telemetry.addData("Angle", Math.toDegrees(angleToGoal1));
        if (USE_TELEMETRY) robot.telemetry.addData("heading current", Math.toDegrees(robotHeading));
        if (USE_TELEMETRY) robot.telemetry.addData("distance to goal",distanceToGoal);
        if (USE_TELEMETRY) robot.telemetry.addData("magnitude",robotVelMagnitude);
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

        double ang = this.calculateTurretAngle(goalPos);
        ang += turretOffsetAngle;
        ang = Math.max(turretLowerBound, Math.min(turretUpperBound, ang));
        this.setTurretAngle(ang);

        if (this.isAutoVelOn) {
            this.setSpeed(inPerSecToRPM(finalVelocity));
        }



        if (this.isAutoHoodOn && robot.robotState != RobotState.SHOOTING) {
            this.setHoodAngle(finalLaunchAngle);
        }

    }


    public void calcHoodPod(Pose2d botPos, Pose2d goalPos, double arcHeight) {
        // note: arcHeight is usually set to the apexHeight variable, which is currently 60
        if (DEBUG) Log.d("ShooterSubsystem", "running hood math");

        double h = arcHeight-robotHeight; // the delta y at the apex

        //my formulas
        double horDist = Math.sqrt(Math.pow((botPos.x-goalPos.x),2) +
                Math.pow((botPos.y-goalPos.y),2)); //simple pythagrean therom
        double verDist = goalHeight - robotHeight; // delta y at the goal
        double theta = Math.atan(((2*h)/horDist) *
                (1 + Math.sqrt(1 - (verDist/h)))); //in radians, from math
        this.setHoodAngle(theta);
        if (DEBUG) Log.d("ShooterSubsystem", "goal hood pos" + this.getHoodServoPos());
    }

    /** lets you set a velocity and angle manually*/
    public void manualAimGoalPos(double velocity, double pitch, Pose2d goalPos) {
        this.isAutoAimOn = false;
        this.setSpeed(this.inPerSecToRPM(velocity));

        this.setHoodAngle(pitch);
        this.setTurretAngle(this.calculateTurretAngle(goalPos));
    }

    public void manualAim(double velocity, double pitch, double turretYaw) {
        this.isAutoAimOn = false;
        Pose2d goalPos = this.goalPosLookupTable.get();

        this.setSpeed(this.inPerSecToRPM(velocity));
        this.setHoodAngle(pitch);
        this.setTurretAngle(this.calculateTurretAngle(goalPos));
    }


    public void manualAimAutoHood (double velocity, double turretYaw) {

        Pose botPosTemp = this.robot.follower.getPose();
        Pose2d botPos = new Pose2d(botPosTemp.getX(), botPosTemp.getY(), botPosTemp.getHeading());
        Pose2d goalPos = this.goalPosLookupTable.get();

        this.isAutoAimOn = false;
        this.setSpeed(inPerSecToRPM(velocity));

        if (this.isAutoHoodOn) {
            calcHoodPod(botPos, goalPos, apexHeight);
        }
        if (USE_TELEMETRY) Robot.debugTelemetry.addData("Calculated Pitch (rad)", this.getHoodAngle());

        if (DEBUG) Log.d("ShooterSubsystem", "Calculated flywheel velocity: " + this.getGoalVelocity() + " rpm");
        if (DEBUG) Log.d("ShooterSubsystem", "Calculated hood pitch (rad)" + this.getHoodAngle());


        this.setTurretAngle(turretYaw);
    }


    private double calculateTurretAngle(Pose2d goalPos){
        double x = goalPos.x - robot.follower.getPose().getX();
        double y = goalPos.y - robot.follower.getPose().getY();
        double absoluteGoalAngle = Math.atan2(y,x);

        double botHeading = robot.follower.getHeading();

        if (USE_TELEMETRY) robot.telemetry.addData("follower heading (deg)",botHeading*180/Math.PI );

        // note: this is 0 to 360 instead of -180 to 180 for convenience below
        return Angle.normalize(absoluteGoalAngle - botHeading);
    }

    public double updateShooter() {
        if (USE_TELEMETRY) Robot.debugTelemetry.addData("Shooter RPM", this.getVelocity());
        if (USE_TELEMETRY) Robot.debugTelemetry.addData("Shooter in/s", this.getVelocity(VelocityUnit.INCHES_PER_SEC));
//        Robot.debugTelemetry.addData("Shooter left (mA)", this.hardware.shooterLeft.getCurrent(CurrentUnit.MILLIAMPS));
//        Robot.debugTelemetry.addData("Shooter right (mA)", this.hardware.shooterRight.getCurrent(CurrentUnit.MILLIAMPS));

        double ff = this.hardware.getVoltageScale() * getGoalVelocity();
        this.shooterPID.setTargetPosition(getGoalVelocity());
        return this.shooterPID.calculatePower(this.getVelocity(), ff);
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
            hardware.shooterPitch.setPosition(this.getHoodServoPos());
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
            double goalTurretPos = turretAngleToServoPos(this.getTurretAngle());
            hardware.turretYawLeft.setPosition(goalTurretPos);
            hardware.turretYawRight.setPosition(goalTurretPos);
            Profiler.pop();
        }
    }
}
