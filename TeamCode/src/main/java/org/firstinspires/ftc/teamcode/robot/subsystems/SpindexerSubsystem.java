package org.firstinspires.ftc.teamcode.robot.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.MathFunctions;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.controllers.PidfController;
import org.firstinspires.ftc.teamcode.robot.hardware.sensors.TerrorColorSensor;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.subsystems.vision.CameraSubsystem;

@Config
public class SpindexerSubsystem extends SubsystemBase {

    private final RobotHardware hardware;
    private final Robot robot;

    public static double INTAKE_WALL_1_DOWN = 1.0;
    public static double INTAKE_WALL_1_UP = 0.05;
    public static double INTAKE_WALL_2_DOWN = 0.85;
    public static double INTAKE_WALL_2_UP = 0.0;

    public static double SHOOTER_RAMP_ACTIVE = 0.3;
    public static double SHOOTER_RAMP_DEACTIVE = 0.03;

    public double intakeWallPosition1 = INTAKE_WALL_1_UP;
    public double intakeWallPosition2 = INTAKE_WALL_2_UP;
    public double shooterRampPosition = SHOOTER_RAMP_DEACTIVE;

    public static double DIDDY_POLE_ACTIVE = 0.2;
    public static double DIDDY_POLE_DEACITVE = 0.4;

    public double diddyPos = DIDDY_POLE_DEACITVE;

    public double spindexerPower = 0.0;
    public TerrorColorSensor[] sensors;

    public static double READY_POSITION = (1D / 6D) * Math.PI; //position for the first ball as the ramp goes down
    double[] yawOffsets = {0, (2.0 / 3) * Math.PI, -((2.0 / 3) * Math.PI)};

    public static PidfController.PidfCoefficients turningPidCoefficients =
            new PidfController.PidfCoefficients(0.3, 0, 0, 0, 0);
    public static double yawPidTolerance = Math.toRadians(10); // radians
    private boolean pidEnabled = true;
    public final PidfController yawPid = new PidfController(turningPidCoefficients);

    public double desiredAngle;

    public SpindexerSubsystem(RobotHardware hardware, Robot robot) {
        this.robot = robot;
        this.hardware = hardware;
        this.sensors = new TerrorColorSensor[]{hardware.rightSensor, hardware.topSensor, hardware.leftSensor};
        this.desiredAngle = getPosition();
        goToAngle120(0);
        this.yawPid.setTolerance(yawPidTolerance);
        this.yawPid.setTargetPosition(0.0);
    }

    public double getPosition() {
        return Angle.angleWrap(hardware.spindexerEncoder.getCurrentPosition());
    }

    public double getTargetYaw() {
        return desiredAngle;
    }

    public boolean atTargetYaw() {
        // TODO: potentially beware of angle wrapping here
        return this.yawPid.atTargetPosition(getPosition());
    }

    /**
     * <p>Snaps the spindexer to a desired angle, while doing modulus to avoid unnecessary rotation.</p>
     * <p>Use this when moving to an angle less than 120 degrees away.</p>
     * @param angle The angle to go to, in radians.
     */
    public void goToAngle120(double angle) {
        double bestAngle = this.desiredAngle;
        double bestError = Double.POSITIVE_INFINITY;

        double sector = 2 * Math.PI / 3.0; // 120 deg

        // basically just loop through the three sides to see which is optimal
        for (int i = 0; i < 3; i++) {
            double equiv = angle + i * sector; // the equivalent angle in our current area
            double error = Angle.angleWrap(equiv - this.desiredAngle);
            if (Math.abs(error) < bestError) {
                bestError = Math.abs(error);
                bestAngle = this.desiredAngle + error;
            }
        }

        this.desiredAngle = bestAngle;
    }


    /**
     * <p>Snaps the spindexer to a desired angle, while doing modulus to avoid unnecessary rotation.</p>
     * <p>Use this when moving to an angle that can be up to 360 degrees away.</p>
     * @param angle The angle to go to, in radians.
     */
    public void goToAngle360(double angle) {
        double error = Angle.angleWrap(angle - this.desiredAngle);
        this.desiredAngle += error;
        Log.d("spindexer", "desired angle gotTo360" + this.desiredAngle);
    }

    /**
     * Increases the desired angle by a certain amount.
     * @param angle The angle to add, in radians.
     */
    public void rotate(double angle) {
        this.desiredAngle += angle;
    }


    public void setWallDown() {
        this.intakeWallPosition1 = INTAKE_WALL_1_DOWN;
        this.intakeWallPosition2 = INTAKE_WALL_2_DOWN;
    }

    public void setWallUp() {
        this.intakeWallPosition1 = INTAKE_WALL_1_UP;
        this.intakeWallPosition2 = INTAKE_WALL_2_UP;
    }

    public void Oilup() {
        this.diddyPos = DIDDY_POLE_ACTIVE;
    }

    public void Oildown() {
        this.diddyPos = DIDDY_POLE_DEACITVE;
    }


    public void enableRamp() {
        shooterRampPosition = SHOOTER_RAMP_ACTIVE;
    }

    public void disableRamp() {
        shooterRampPosition = SHOOTER_RAMP_DEACTIVE;
    }


    public char[] getBallPositions() {
        return new char[]{hardware.topSensor.getGreenOrPurple(), hardware.rightSensor.getGreenOrPurple(), hardware.leftSensor.getGreenOrPurple()};
    }

    public double selectColor(char color) {
        int nearestIndex = new String(getBallPositions()).indexOf(color);
        return yawOffsets[nearestIndex];
    }


    public void sortBalls() {
        Log.d("spindexer", "des ang before sort "+this.desiredAngle);
        this.goToAngle120(0);
        Log.d("spindexer", "des ang aft 0 "+this.desiredAngle);
        int fullCount = 0;
        double greenPos = 0.0;
        int greenCount = 0;
        int purpleCount = 0;
        for (char ball : this.getBallPositions()) {
            Log.d("ball-thing", String.valueOf(ball));
            if (ball!= 'N') {
                fullCount += 1;
                if (ball == 'G') {
                    greenCount += 1;
                    greenPos = this.selectColor('G');
                } else {
                    purpleCount += 1;
                }

            }
        }
        Log.d("spindexer", "purple count" + purpleCount);
        Log.d("spindexer", "green count" + greenCount);
        Log.d("spindexer", "full count" + fullCount);
        Log.d("spindexer", "greenPos" + greenPos);

        if (purpleCount == 2 && greenCount == 1) {
            if (robot.camera.gameGlyph == CameraSubsystem.GLYPH.GPP) {
                double normalizedError = MathUtils.normalizeRadians(-(READY_POSITION - greenPos), false);
                Log.d("spindexer", "glyph gpp normalized error" + normalizedError);
                this.rotate(normalizedError);

            } else if (robot.camera.gameGlyph == CameraSubsystem.GLYPH.PGP) {
                double normalizedError = MathUtils.normalizeRadians(-((READY_POSITION + ((2D / 3D) * Math.PI)) - greenPos), false);
                Log.d("spindexer", "glyph pgp normalized error" + normalizedError);
                this.rotate(normalizedError);

            } else {
                double normalizedError = MathUtils.normalizeRadians(-((READY_POSITION + ((4D / 3D) * Math.PI)) - greenPos), false);
                Log.d("spindexer", "glyph ppg normalized error" + normalizedError);
                this.rotate(normalizedError);
            }
        } else {
            Log.d("spindexer", "not enough balls to run logic ready pos:" + READY_POSITION);
            this.rotate(READY_POSITION);
        }
        Log.d("spindexer", "des ang after sort"+this.desiredAngle);

    }

    /**
     * <p>Directly sets the angle of the spindexer.</p>
     * <p>This is deprecated, and only exists for the tuning files.
     * See {@link #goToAngle120(double)}, {@link #goToAngle360(double)}, and {@link #rotate(double)} for better alternatives.</p>
     * @param angle The angle to set the spindexer to.
     */
    @Deprecated
    public void setYaw(double angle) { //angle is in radians cuz i said so oh yeah and also have todo: optimization like the swerve pod thingy where u do the shortest distance
        this.desiredAngle = angle;
    }

    public void setSpindexerPower(double power) {
        this.spindexerPower = power;
    }

    public void setPidEnabled(boolean enabled) {
        spindexerPower = 0.0;
        pidEnabled = enabled;
    }

    public void updateSpindexer() {
        // setTargetPosition as 0.0 is intentional since PID does not account for angle wrapping, so
        // we calculate error ourselves and feed into PID.
        this.yawPid.setTargetPosition(0.0);
        if (pidEnabled) {
            double error = MathFunctions.getSmallestAngleDifference(desiredAngle, getPosition()) * MathFunctions.getTurnDirection(getPosition(), desiredAngle);
            this.spindexerPower = yawPid.calculatePower(error, 0);
        }

    }



    @Override
    public void periodic() {


        if (robot.hang.isPtoEngaged()) {
            return;
        }

        this.updateSpindexer();
        this.hardware.spindexerRotate.setPower(this.spindexerPower);
        this.hardware.spindexerIntakeWallServo1.setPosition(intakeWallPosition1);
        this.hardware.spindexerIntakeWallServo2.setPosition(intakeWallPosition2);
        this.hardware.spindexerDiddyServo.setPosition(diddyPos);
        this.hardware.spindexerTransferRampServo.setPosition(shooterRampPosition);

        Robot.debugTelemetry.addData("Spindexer Position (deg)", Math.toDegrees(Angle.angleWrap(getPosition())));
        Robot.debugTelemetry.addData("Spindexer Target (deg)", Math.toDegrees(Angle.angleWrap(getTargetYaw())));
    }
}
