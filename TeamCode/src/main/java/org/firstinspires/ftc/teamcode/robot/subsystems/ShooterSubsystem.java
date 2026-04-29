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
import org.firstinspires.ftc.teamcode.math.datastructures.CircularBuffer;
import org.firstinspires.ftc.teamcode.pedroPathing.FtcDashDrawing;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;
import org.firstinspires.ftc.teamcode.robot.subsystems.shooter.FlightTimeLookupTable;
import org.firstinspires.ftc.teamcode.robot.subsystems.shooter.GoalPosLookupTable;
import org.firstinspires.ftc.teamcode.robot.subsystems.shooter.ShooterLookupTable;
import org.firstinspires.ftc.teamcode.robot.subsystems.shooter.ShooterLookupTableInstance;
import org.firstinspires.ftc.teamcode.util.Profiler;

@Config
public class ShooterSubsystem extends SubsystemBase {
    public static int ACCEL_BUFFER_SZE = 3;
    public static double ACCELERATION_COEFFICIENT = 0.05;
    public static boolean USE_SOTM = true;
    public static boolean USE_SOTM_ACCEL = false;
    public static boolean JUST_TURRET = false;

    public static boolean debug = false;
    public static boolean telemetry = true;

    public static double TICKS_PER_REV = 28; // GoBilda yellowjacket encoder

    // Small/large PID is for when current velocity is small/large, respectively.
    // From my observations, the flywheel is more sensitive at lower velocities, so we use a less
    // aggressive feedforward at higher velocities to avoid overshooting and oscillation.
    public static PidfController.PidfCoefficients NEAR_PID_COEFFICIENTS =
            new PidfController.PidfCoefficients(0.0007, 0, 0, 0.000225, 0);
    public static PidfController.PidfCoefficients FAR_PID_COEFFICIENTS =
            new PidfController.PidfCoefficients(0.0007, 0, 0, 0.000207, 0);
    private final PidfController shooterPID = new PidfController(NEAR_PID_COEFFICIENTS);

    // SHOOTER_PID_SWITCH determines when we switch between the two PIDs.
    public static double SHOOTER_PID_SWITCH = 2500;  // Units are RPM

    // SHOOTER_VEL_TOLERANCE determines when we consider the shooter to be "at velocity"
    public static double SHOOTER_VEL_TOLERANCE = 1000;  // Units are RPM
    public static double SHOOTER_VEL_MAXPOWER_TOLERANCE = 500;// Units are RPM, used for quicker recovery while shooting multiple balls
    public static double POWER_ADD = 0.3;

    public GoalPosLookupTable goalPosLookupTable;
    public ShooterLookupTableInstance shooterLookupTable = ShooterLookupTable.NORMAL_TABLE;

    // what the shooter should be at
    public double goalPitch; //hood - rad
    public double goalVelocity; //flywheel - rpm
    public double goalTurretAngle; //turret - rad
    public double goalPitchPos; //hood - servo pos todo: remove, only have goalPitch

    // turret positions
    public static double turretOffset = 0.00; //turret manual offset- servo pos
    public static double turretPosAt180 = 0.5225; //pos pointed directly towards the back
    public static double posChange90 = 0.280; //servo pos change that rotates turret 90 deg
    public static double posChange90Right = 0.2845; //servo pos change that rotates turret 90 deg
    public static double posChange90Left = 0.28; //servo pos change that rotates turret 90 deg
    public static double turretServosDifferenceSmall = 0.001; // we set the two servos to positions of +- 0.02 to reduce backlash by making them fight
    public static double turretServosDifferenceLarge = 0.01;
    public static Coordinate turretToRobotCenterOffset = new Coordinate(-1.61417, 0);

    // in loops, how often to update the turret position servo when outside of the shooting zone
    public static double TURRET_UPDATE_FREQUENCY = 1; // todo: set this to 10 later
    private double loopCount = 0;

    // No angle limit for turret, but we have servo positions limits
    public static double turretLowerBound = Math.toRadians(0);
    public static double turretUpperBound = Math.toRadians(360);
    public static double turretServoLowerBound = 0.03;
    public static double turretServoUpperBound = 0.97;

    // hood limits
    public static double hoodPosMax = 0.98; //maximum position the servo can go to
    public static double hoodPosMin = 0.2; //min position the servo can go to
    public static double hoodAngleMax = 1.0; //radian measure of hood at max pos
    public static double hoodAngleMin = 0.05; //radian measure of hood at min pos // todo: this being 0.05 is temporary since the hood keeps getting cooked at the bottom, please set it back to 0 eventually

    // vars for calculating shot (unused currently, todo: remove later)
    public static double robotHeight = 14.0; //in
    public static double g = 386.08858267717; //in per sec^2
    public static double goalHeight = 40.0; //doesnt change no matter alliance color
    public static final double GOAL_HEIGHT_RETURN = goalHeight;
    public static double apexHeight = 60.0; //what the apex of the balls path is going to try to be

    private Pose2d center = new Pose2d(72,72,0);

    public double goalPosVerticalOffset = 0;
    public double goalPosHorizontalOffset = 0;

    Pose2d horizontalOffet = new Pose2d(0,0,0);
    Pose2d verticalOffset = new Pose2d(0,0,0);

    private final RobotHardware hardware;
    private final Robot robot;

    // flags for autoshoot
    public boolean disableFlywheel = false;
    public boolean isAutoAimOn;
    public boolean isAutoVelOn;
    public boolean isAutoHoodOn;
    public boolean isAutoTurretOn;
    public boolean alwaysUpdateTurret = false;

    public static int rollingValLen = 5;
//    public LinkedHashMap<Double, Double> velValues;
    public int ballsShot = 0;
    public ElapsedTime ballsShotTimer = new ElapsedTime();

    // If this is set, the robot will use this pose instead of the follower pose for auto-shoot
    // calculations. If null, the follower pose is used.
    //
    // This is useful during auto to avoid dynamically updating the shooter while the robot moves.
    public Pose autoShootPoseOverride = null;
    public Boolean sotmOverride = null;
    public Boolean sotmAccelOverride = null;

    // flag used for lighting feedback for driver
    public boolean turretInDeadzone = false;

    private CircularBuffer<Double> accelBufferX = new CircularBuffer<>(ACCEL_BUFFER_SZE);
    private CircularBuffer<Double> accelBufferY = new CircularBuffer<>(ACCEL_BUFFER_SZE);

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

        this.shooterPID.setTargetPosition(0.0);

        this.goalPosLookupTable = new GoalPosLookupTable(this.robot);
//        for (int i=0; i < rollingValLen; i++){
//            velValues.put(0D, 0D);
//        }



        //currently doesnt control anything in this class, just for keeping track
        this.isAutoAimOn = true;
        this.isAutoVelOn = true;
        this.isAutoHoodOn = true;
        this.isAutoTurretOn = true;
        this.turretInDeadzone = false;
    }

    public static double turretAngleToServoPos(double angleRad) {
        double unboundedServo;
        if (angleRad <= 180){
             unboundedServo = Algebra.mapRangeNoClamp(angleRad,
                    Math.toRadians(90), Math.toRadians(180),
                    turretPosAt180-posChange90Right, turretPosAt180
            );
        }
        else{
            unboundedServo = Algebra.mapRangeNoClamp(angleRad,
                    Math.toRadians(180), Math.toRadians(270),
                    turretPosAt180, turretPosAt180+posChange90Left
            );
        }

        return MathFunctions.clamp(unboundedServo, turretServoLowerBound, turretServoUpperBound);
    }

    public void setGoalPosOffset(double vertical, double horizontal)
    {
        this.goalPosVerticalOffset = vertical;
        this.goalPosHorizontalOffset = horizontal;
        goalHeight = GOAL_HEIGHT_RETURN;
    }

    public void resetGoalPosOffset()
    {
        setGoalPosOffset(0,0);
    }

    public void incrementGoalPosOffset(double vertical, double horizontal)
    {
        this.goalPosVerticalOffset += vertical;
        this.goalPosHorizontalOffset += horizontal;
        goalHeight += vertical;
    }

    public Pose2d recalculateGoalPosWithOffsets(Pose2d goalPos)
    {
        if(goalPosHorizontalOffset == 0 && goalPosVerticalOffset == 0)
        {
            return goalPos;
        }
        double goalPosMagnitude = Math.hypot(goalPos.x-center.x,goalPos.y-center.y);
        Pose2d normalizedGoalPos = new Pose2d((goalPos.x-center.x)/ goalPosMagnitude,(goalPos.y-center.y) / goalPosMagnitude);
        double xShift = goalPosHorizontalOffset *normalizedGoalPos.x;
        double yShift = goalPosVerticalOffset *normalizedGoalPos.y;

        verticalOffset.x = xShift;
        verticalOffset.y = yShift;

        horizontalOffet.x = yShift;
        horizontalOffet.y = -xShift;

        return goalPos.plus(horizontalOffet).plus(verticalOffset);
    }

    public void doAutoShoot(Pose botPos, boolean useVelocityCompensation, boolean useAccelCompensation) {
        if (debug) Log.d("ShooterSubsystem", "Doing autoshoot!");
        this.isAutoAimOn = true;

        Pose2d goalPos = this.goalPosLookupTable.getForPose(botPos);
        goalPos = recalculateGoalPosWithOffsets(goalPos);
        Pose2d oldGoalPos = goalPos.copy();
        //Pose2d goalPos = this.robot.color.getGoalPos();
        double distToGoal = botPos.distanceFrom(goalPos.toPedro());
        double oldDistToGoal = distToGoal;
        FtcDashDrawing.drawDot(goalPos.toPedro(), "#000000");


        if (useVelocityCompensation) {
            double flightTime = FlightTimeLookupTable.get(distToGoal);
            Vector robotVel = robot.follower.getVelocity();
            if (useAccelCompensation) {
                Vector accel = robot.follower.getAcceleration();
                accelBufferX.add(accel.getXComponent());
                accelBufferY.add(accel.getYComponent());

                Vector avgAccel = new Vector();
                avgAccel.setOrthogonalComponents(accelBufferX.getMean(), accelBufferY.getMean());
                robotVel = robotVel.plus(avgAccel.times(ACCELERATION_COEFFICIENT));
            }
            Vector goalAdjAmt = robotVel.times(flightTime);
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
        if (JUST_TURRET){
            math = shooterLookupTable.get(oldDistToGoal);
        }
        else math = shooterLookupTable.get(distToGoal);
        //calcVelcoity - in/sec

        if (telemetry) Robot.debugTelemetry.addData("Calculated Velocity (in/sec)", math.velocity);


        //velocity is in inches/second, if this doesnt match the encoder we'll have to fix
        if (this.isAutoVelOn) {
            this.setSpeed(this.velToRPM(math.velocity)); // todo: add back
        }
        if (this.isAutoHoodOn && !(robot.getState() == RobotState.SHOOTING && !USE_SOTM_ACCEL)) {
            this.goalPitch = math.rad;
            this.goalPitchPos = Algebra.mapRange(math.rad, hoodAngleMin, hoodAngleMax, hoodPosMin, hoodPosMax);
        }
        if (telemetry) Robot.debugTelemetry.addData("Calculated Pitch (rad)", this.goalPitch);

        if (debug) Log.i("ShooterSubsystem", "Calculated flywheel velocity: " + this.getGoalVelocity() + " rpm");
        if (debug) Log.i("ShooterSubsystem", "Calculated hood pitch (rad)" + this.goalPitch);
    }

    public void intermediateAim(Pose botPos, boolean useVelocityCompensation){
        this.isAutoAimOn = false;
        this.setSpeed(this.goalVelocity);

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

        if (isAutoTurretOn && (alwaysUpdateTurret || loopCount == 0 || robot.isInTapeZone())) {
            this.setTurretAngle(this.findYawAngle(botPos, goalPos));
        }

        ShooterValues math;
        math = shooterLookupTable.get(distToGoal);

        if (this.isAutoHoodOn && robot.robotState != RobotState.SHOOTING) {
            this.goalPitch = math.rad;
            this.goalPitchPos = Algebra.mapRange(math.rad, hoodAngleMin, hoodAngleMax, hoodPosMin, hoodPosMax);
        }
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

    public void manualAimAutoTurret(double velocity, double pitch) {
        this.isAutoAimOn = false;
        this.setSpeed(this.velToRPM(velocity));

        this.goalPitch = pitch;
        this.goalPitchPos = Algebra.mapRange(pitch, hoodAngleMin, hoodAngleMax, hoodPosMin, hoodPosMax);

        this.setTurretAngle(this.findYawAngle(this.robot.follower.getPose(), recalculateGoalPosWithOffsets(goalPosLookupTable.get())));
    }

    public void manualAim(double velocity, double pitch, double turretYaw) {
        Pose2d goalPos = this.goalPosLookupTable.get();

        this.isAutoAimOn = false;
        this.setSpeed(this.velToRPM(velocity));

        this.goalPitch = pitch;
        this.goalPitchPos = Algebra.mapRange(pitch, hoodAngleMin, hoodAngleMax, hoodPosMin, hoodPosMax);

        this.setTurretAngle(turretYaw);
    }

    private double findYawAngle(Pose botPos, Pose2d goalPos){
        /** all in rad **/
//        Pose robotVector = new Pose(turretToRobotCenterOffset.x, turretToRobotCenterOffset.y, 0)
//                .rotate(botPos.getHeading(), false);
//        Pose turretCenter = botPos.plus(robotVector);
        double dx = goalPos.x - botPos.getX();
        double dy = goalPos.y - botPos.getY();
        double angle = Math.atan2(dy, dx);

        double absoluteGoalAngle = angle;
        Robot.debugTelemetry.addData("absolute goal angle (deg)", Math.toDegrees(absoluteGoalAngle));



        double botHeading = botPos.getHeading();
        if (telemetry) robot.telemetry.addData("follower heading (deg)",botHeading*180D/Math.PI );

        double angleTurret = Angle.normalize(absoluteGoalAngle - botHeading);

        if (debug) Log.d("ShooterSubsystem", "turret angle (deg): " + Math.toDegrees(angleTurret));
        if (debug) Log.d("ShooterSubsystem", "calculated servo pos: " + turretAngleToServoPos(angleTurret));

        Pose turretPose = new Pose(botPos.getX(), botPos.getY(), botPos.getHeading() + angleTurret);
        FtcDashDrawing.drawRobot(turretPose, "#FFFFFF");
        FtcDashDrawing.drawHeadingRay(turretPose, "FFFFFF");
        return angleTurret;
    }

//    public void updateRollingVelValues(){
//        velValues.remove(velValues.keySet().iterator().next());
//        velValues.put(this.getVelocity(), this.getGoalVelocity());
//    }
//
//    public void checkShotBalls(){
//        boolean valid = true;
//        if (velValues.get(velValues.keySet().iterator().next()) - goalVelocity < 20){
//
//            List<Double> differences = new ArrayList<>();
//            for (Map.Entry<Double, Double> entry : velValues.entrySet()){
//                 differences.add(Math.abs(entry.getKey())-entry.getValue());
//            }
//            double avg = 0;
//
//            for (double d1: differences){
//                for (double d2: differences){
//                    if (Math.abs(d1-d2) > 10){
//                        valid = false;
//                    }
//                }
//                avg += d1;
//            }
//            avg = avg/rollingValLen;
//
//            if (valid && Math.abs(goalVelocity-this.getVelocity()) *2 > avg){
//                ballsShot +=1;
//                Log.i("ShooterSubsystem.java", "Ball Shot!");
//                ballsShotTimer.reset();
//            }
//
//        }
//    }
//    public int getBallsShot(){return ballsShot;}



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

    public static double rpmToVel(double rpm){
        return rpm / 6.469;
    }

    public boolean isFlywheelAtTarget() {
        return shooterPID.atTargetPositionWithTolerance(this.getVelocityRpm(), SHOOTER_VEL_TOLERANCE);
    }

    public double updateShooter() {
        if (disableFlywheel) {
            if (telemetry) {
                Robot.debugTelemetry.addData("Calculated Velocity (in/sec)", 0.0);
                Robot.debugTelemetry.addData("Shooter in/s", 0.0);
            }
            return 0.0;
        }

        double currentRpm = this.getVelocityRpm();
        if (telemetry) Robot.debugTelemetry.addData("Shooter RPM", currentRpm);
        if (telemetry) Robot.debugTelemetry.addData("Shooter in/s", currentRpm / 6.469);

        shooterPID.setTargetPosition(getGoalVelocity());
//        boolean useSmallPID = shooterPID.atTargetPositionWithTolerance(currentRpm, SHOOTER_PID_SWITCH);
        boolean useSmallPID = currentRpm < SHOOTER_PID_SWITCH;
        shooterPID.setPidfCoefficients(useSmallPID ? NEAR_PID_COEFFICIENTS : FAR_PID_COEFFICIENTS);

       // double shooterPower = 0.0;
        double distToGoal;
        if (robot.color == null){
            distToGoal = 0;
        }
        else{
            distToGoal = robot.follower.getPose().distanceFrom(robot.color.getGoalPos().toPedro());

        }
        double shooterPower = hardware.getVoltageScale() * shooterPID.calculatePower(currentRpm, getGoalVelocity(), false);
        if ((getGoalVelocity() - currentRpm < SHOOTER_VEL_MAXPOWER_TOLERANCE)
                && (getGoalVelocity() - currentRpm > 0) &&
                robot.getState().equals(RobotState.SHOOTING)
                && distToGoal > 100) {
            shooterPower += POWER_ADD; // if we're too far below the target, just go full power to get there faster
        }
//        else if (currentRpm - getGoalVelocity() > SHOOTER_VEL_MAXPOWER_TOLERANCE) {
//            shooterPower = 0;
//        }
        return shooterPower;
    }

    public void addTurretOffset(double change){
        this.turretOffset += change;
    }

    public double getGoalTurretYaw() {
        return this.goalTurretAngle;
    }

    public void toggleSOTMOverride()
    {
        if(sotmOverride == null)
        {
            sotmOverride = true;
            return;
        }
        sotmOverride = !sotmOverride;
    }

    @Override
    public void periodic() {
        try (Profiler.Scope p = Profiler.enter("ShooterSubsystem")) {
            if (robot.robotState.isHang()) {
                hardware.shooterLeft.setPower(0);
                hardware.shooterRight.setPower(0);
                double turretYaw = turretAngleToServoPos(Math.toRadians(180.0)) + this.turretOffset;
//                hardware.turretYawLeft.setPosition(turretYaw);
//                hardware.turretYawRight.setPosition(turretYaw);
                return;
            }

            Profiler.push("ball shot logic");
//            updateRollingVelValues();
//            if (robot.robotState == RobotState.TRANSFER || robot.robotState == RobotState.SHOOTING){
//                checkShotBalls();
//            }
            if (ballsShotTimer.seconds() > 3 && ballsShot >0){
                ballsShot = 0;
            }
            Profiler.pop();



            Profiler.push("autoshoot");
            loopCount = (loopCount + 1) % TURRET_UPDATE_FREQUENCY;
            if (robot.goalPos != null && isAutoAimOn) {
                Pose robotPos = this.robot.follower.getPose();
                boolean useSotm = sotmOverride != null ? sotmOverride : USE_SOTM;
                boolean useSotmAccel = sotmAccelOverride != null ? sotmAccelOverride : USE_SOTM_ACCEL;
                this.doAutoShoot(robotPos, useSotm, useSotmAccel);
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
            Robot.debugTelemetry.addData("Goal Turret Angle (deg)", Math.toDegrees(this.goalTurretAngle));
            double goalTurretPos = turretAngleToServoPos(this.goalTurretAngle) + this.turretOffset;
            this.turretInDeadzone = (goalTurretPos <= turretServoLowerBound) || (goalTurretPos >= turretServoUpperBound);
            double turretServosDifference = (goalTurretPos < 0.3) ? turretServosDifferenceLarge : turretServosDifferenceSmall;

            boolean useDifferenceForBacklash = goalTurretPos + turretServosDifference <= turretServoUpperBound && goalTurretPos - turretServosDifference >= turretServoLowerBound;
            double difference = useDifferenceForBacklash ? turretServosDifference : 0;
            hardware.turretYawLeft.setPosition(goalTurretPos + difference);
            hardware.turretYawRight.setPosition(goalTurretPos - difference);
            Profiler.pop();

            if (debug) Log.d("ShooterSubsystem", "goal hood angle" + this.goalPitch);
            if (debug) Log.d("ShooterSubsystem", "goal hood pos" + this.goalPitchPos);
        }
    }
}
