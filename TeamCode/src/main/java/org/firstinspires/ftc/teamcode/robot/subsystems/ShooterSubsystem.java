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
import org.firstinspires.ftc.teamcode.robot.subsystems.shooter.HardCodedLookup;
import org.firstinspires.ftc.teamcode.robot.subsystems.shooter.ShooterLookupTable;

@Config
public class ShooterSubsystem extends SubsystemBase {
    private final RobotHardware hardware;

    public static boolean usingHardCodedShooterTable = false;
    public static double TICKS_PER_REV = 28; // GoBilda yellowjacket encoder

    // TODO: tune velocity pid coefficients + tolerance
    public static PidfController.PidfCoefficients shooterPIDCoeffecients =
            new PidfController.PidfCoefficients(0.0001, 0.000115, 0.00, 0, 0);
    public static double SHOOTER_VELOCITY_TOLERANCE = 0.0;

    // the current pid + speed
    public final PidfController shooterPID = new PidfController(shooterPIDCoeffecients);
    public double shooterPower = 0.0; //flywheel - motor power

    public static double turretPosAt180 = 0.54; //pos pointed directly towards the back
    public static double posChange90 = 0.38; //servo pos change that rotates turret 90 deg

    private double goalPitch; //hood - rad
    private double goalVelocity; //flywheel - rpm
    public double goalTurretAngle; //turret - rad

    public double goalTurretPos; //turret - servo pos
    private double goalPitchPos; //hood - servo pos

    public double turretOffset = 0.0; //turret manual offset- servo pos

    public static double turretLowerBound = Math.PI/2; //currently 90 deg, var in rad
    public static double turretUpperBound = 3*Math.PI/2; //currently 270 deg, var in rad

    public static double hoodPosMax = 0.35; //maximum position the servo can go to
    public static double hoodPosMin = 0.55; //min position the servo can go to
    public static double hoodAngleMax = 1.2217; //radian measure of hood at max pos
    public static double hoodAngleMin = 0.8726; //radian measure of hood at min pos

    public boolean isAutoAimOn;
    public boolean isAutoVelOn;
    public boolean isAutoHoodOn;
    private final Robot robot;

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
        Log.i("shooter", "Doing autoshoot!");
        this.isAutoAimOn = true;

        Pose botPosTemp = this.robot.follower.getPose();
        Pose2d botPos = new Pose2d(botPosTemp.getX(), botPosTemp.getY(), botPosTemp.getHeading());

        //currently limited to 90 - 270 degrees, can be changed by changing the values in the map range below
        this.goalTurretAngle = this.findYawAngle(goalPos);
        this.goalTurretPos = Algebra.mapRange(this.goalTurretAngle, turretLowerBound, turretUpperBound, turretPosAt180-posChange90, turretPosAt180+posChange90);


        ShooterValues math = ShooterLookupTable.get(botPos.toPedro().distanceFrom(goalPos.toPedro()));
        if(usingHardCodedShooterTable)
        {
            math = HardCodedLookup.get(botPos.toPedro().distanceFrom(goalPos.toPedro()));
        }

        if (math.flywheelVelocity == null || math.hoodPitch == null) {
            Log.e("shooter", "failed to do math!");
            return;
        }
        //math.flywheelVelocity - in/sec
        //math.hoodPitch - rad

        Robot.debugTelemetry.addData("Calculated Velocity (in/sec)", math.flywheelVelocity);
        Robot.debugTelemetry.addData("Calculated Pitch (rad)", math.hoodPitch);

        //velocity is in inches/second, if this doesnt match the encoder we'll have to fix
        if (this.isAutoVelOn) {
            this.setSpeed(this.velToRPM(math.flywheelVelocity)); // todo: add back
        }
        if (this.isAutoHoodOn) {
            //gets a setpos from the angle from our measured angles for max and min
            this.goalPitch = math.hoodPitch;
            this.goalPitchPos = Algebra.mapRange(math.hoodPitch, hoodAngleMin, hoodAngleMax, hoodPosMin, hoodPosMax);
        }

        Log.i("shooter", "Calculated flywheel velocity: " + this.getGoalVelocity() + " rpm");
        Log.i("shooter", "Calculated hood pitch (rad)" + this.goalPitch);
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


    public static class ShooterValues {
        public Double flywheelVelocity;
        public Double hoodPitch;

        public ShooterValues(Double flywheelVelocity, Double hoodPitch) {
            this.flywheelVelocity = flywheelVelocity;
            this.hoodPitch = hoodPitch;
        }
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
        this.shooterPower = this.shooterPID.calculatePower(this.getVelocityRpm(),0);
    }

    public void addTurretOffset(double change){
        this.turretOffset += change;
    }


    @Override
    public void periodic() {
        if (robot.goalPos != null && isAutoAimOn) this.doAutoShoot(robot.goalPos);
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
