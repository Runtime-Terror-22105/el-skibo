package org.firstinspires.ftc.teamcode.robot.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.math.Algebra;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.math.controllers.PidfController;
import org.firstinspires.ftc.teamcode.robot.subsystems.shooter.GoalPosLookupTable;
import org.firstinspires.ftc.teamcode.robot.subsystems.shooter.HardCodedLookup;
import org.firstinspires.ftc.teamcode.robot.subsystems.shooter.ShooterLookupTable;

@Config
public class ShooterSubsystem extends SubsystemBase {
    private final RobotHardware hardware;

    public static boolean usingHardCodedShooterTable = false;
    public static double TICKS_PER_REV = 28; // GoBilda yellowjacket encoder

    // TODO: tune velocity pid coefficients + tolerance
    public static PidfController.PidfCoefficients shooterPIDCoeffecients =
            new PidfController.PidfCoefficients(0.0002, 0.0, 0.0, 0.00017, 0);
    public static double SHOOTER_VELOCITY_TOLERANCE = 0.0;

    // the current pid + speed
    public final PidfController shooterPID = new PidfController(shooterPIDCoeffecients);
    public double shooterPower = 0.0; //flywheel - motor power

    public static double turretPosAt180 = 0.47; //pos pointed directly towards the back
    public static double posChange90 = 0.38; //servo pos change that rotates turret 90 deg

    public double goalPitch; //hood - rad
    public double goalVelocity; //flywheel - rpm
    public double goalTurretAngle; //turret - rad

    public double goalTurretPos; //turret - servo pos
    public double goalPitchPos; //hood - servo pos

    public double turretOffset = 0.0; //turret manual offset- servo pos

    public GoalPosLookupTable goalPosLookupTable;

    public static double turretLowerBound = Math.PI/2; //currently 90 deg, var in rad
    public static double turretUpperBound = 3*Math.PI/2; //currently 270 deg, var in rad

    public static double hoodPosMax = 0.85; //maximum position the servo can go to
    public static double hoodPosMin = 0.2; //min position the servo can go to
    public static double hoodAngleMax = 0.919427826056; //radian measure of hood at max pos
    public static double hoodAngleMin = 0.632748891943; //radian measure of hood at min pos
    public static double robotHeight = 14.0; //in
    public static double g = 386.08858267717; //in per sec^2
    public static double goalHeight = 40.0; //doesnt change no matter alliance color
    public static double apexHeight = 60.0; //what the apex of the balls path is going to try to be

    public boolean isAutoAimOn;
    public boolean isAutoVelOn;
    public boolean isAutoHoodOn;
    private final Robot robot;

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

    }

    public void doAutoShoot(){
        Log.i("shooter", "Doing autoshoot!");
        this.isAutoAimOn = true;

        Pose botPosTemp = this.robot.follower.getPose();
        Pose2d botPos = new Pose2d(botPosTemp.getX(), botPosTemp.getY(), botPosTemp.getHeading());
        Pose2d goalPos = this.goalPosLookupTable.get();

        //currently limited to 90 - 270 degrees, can be changed by changing the values in the map range below
        this.goalTurretAngle = this.findYawAngle(goalPos);
        this.goalTurretPos = Algebra.mapRange(this.goalTurretAngle, turretLowerBound, turretUpperBound, turretPosAt180-posChange90, turretPosAt180+posChange90);

        double calcVelocity;
        if(usingHardCodedShooterTable)
        {
            calcVelocity = HardCodedLookup.get(botPos.toPedro().distanceFrom(goalPos.toPedro()));
        }
        else{
            calcVelocity = ShooterLookupTable.get(botPos.toPedro().distanceFrom(goalPos.toPedro()));
        }
        //calcVelcoity - in/sec

        Robot.debugTelemetry.addData("Calculated Velocity (in/sec)", calcVelocity);


        //velocity is in inches/second, if this doesnt match the encoder we'll have to fix
        if (this.isAutoVelOn) {
            this.setSpeed(this.velToRPM(calcVelocity)); // todo: add back
        }
        if (this.isAutoHoodOn) {
            calcHoodPod(botPos, goalPos, apexHeight);
        }
        Robot.debugTelemetry.addData("Calculated Pitch (rad)", this.goalPitch);

        Log.i("shooter", "Calculated flywheel velocity: " + this.getGoalVelocity() + " rpm");
        Log.i("shooter", "Calculated hood pitch (rad)" + this.goalPitch);
    }

    public void calcHoodPod(Pose2d botPos, Pose2d goalPos, double arcHeight){
        Log.d("shooter", "running hood math");

        double h = arcHeight;

        //my formulas
        double horDist = Math.sqrt(Math.pow((botPos.x-goalPos.x),2) +
                Math.pow((botPos.y-goalPos.y),2)); //simple pythagrean therom
        double verDist = goalHeight - robotHeight;
        double theta = Math.atan(((2*h)/horDist) *
                (1 + Math.sqrt(1 - (verDist/h)))); //in radians, from math
        Log.d("shooter", "goal hood angle" + theta);
        this.goalPitch = theta;
        this.goalPitchPos = Algebra.mapRange(theta, hoodAngleMin, hoodAngleMax, hoodPosMin, hoodPosMax);
        Log.d("shooter", "goal hood pos" + this.goalPitchPos);
    }

    /** lets you set a velocity and angle manually*/
    public void manualAim(double velocity, double pitch, double turretYaw) {

        this.isAutoAimOn = false;
        this.setSpeed(this.velToRPM(velocity));

        this.goalPitch = pitch;
        this.goalPitchPos = Algebra.mapRange(pitch, hoodAngleMin, hoodAngleMax, hoodPosMin, hoodPosMax);

        this.goalTurretAngle = Math.max(turretLowerBound, Math.min(turretUpperBound, turretYaw));
        this.goalTurretPos = Algebra.mapRange(this.goalTurretAngle, turretLowerBound, turretUpperBound, turretPosAt180-posChange90, turretPosAt180+posChange90);

    }


    private double findYawAngle(Pose2d goalPos){
         /** all in rad **/
         double x = goalPos.x - robot.follower.getPose().getX();
         double y = goalPos.y - robot.follower.getPose().getY();
         double angle = Math.atan2(y,x);

         double absoluteGoalAngle = angle;

         double botHeading = robot.follower.getHeading();

        robot.telemetry.addData("follower heading (deg)",botHeading*180/Math.PI );


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
        Log.d("shooter", "setSpeed (rpm): " + goal);
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
        Robot.debugTelemetry.addData("Shooter RPM", this.getVelocityRpm());
        Robot.debugTelemetry.addData("Shooter in/s", this.getVelocityRpm() / 6.469);
//        Robot.debugTelemetry.addData("Shooter left (mA)", this.hardware.shooterLeft.getCurrent(CurrentUnit.MILLIAMPS));
//        Robot.debugTelemetry.addData("Shooter right (mA)", this.hardware.shooterRight.getCurrent(CurrentUnit.MILLIAMPS));
        this.shooterPID.setTargetPosition(getGoalVelocity());
        this.shooterPower = this.shooterPID.calculatePower(this.getVelocityRpm(), getGoalVelocity());
    }

    public void addTurretOffset(double change){
        this.turretOffset += change;
    }


    @Override
    public void periodic() {
        if (robot.goalPos != null && isAutoAimOn) this.doAutoShoot();
        else Log.e("ShooterSubsystem", "robot.goalPos is null! Skipping autoshoot...");

        // shooter pitch
        hardware.shooterPitch.setPosition(this.goalPitchPos);

        // flywheel pids
        this.updateShooter();
        Robot.debugTelemetry.addData("Shooter Power", shooterPower);
        hardware.shooterLeft.setPower(shooterPower);
        hardware.shooterRight.setPower(shooterPower);

        //turret
        hardware.turretYawLeft.setPosition(this.goalTurretPos + this.turretOffset);
        hardware.turretYawRight.setPosition(this.goalTurretPos + this.turretOffset);
    }
}
