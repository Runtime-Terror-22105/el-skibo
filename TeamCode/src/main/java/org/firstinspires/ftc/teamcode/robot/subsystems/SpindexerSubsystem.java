package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
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
    private enum COLOR {
        GREEN, PURPLE
    }

    private final RobotHardware hardware;
    private final Robot robot;

    private double homedSpindexerOffset = 0;

    public static double SPINDEXER_OFFSET_2 = 0;

    public static double TICKS_PER_REVOLUTION = ((1D + (46D / 11D)) * 28D) * 5.6D;

    public static double INTAKE_WALL_1_DOWN = 0.345;
    public static double INTAKE_WALL_1_UP = 0.7;
    public static double INTAKE_WALL_2_DOWN = 0.555;
    public static double INTAKE_WALL_2_UP = 0.23;

    public static double SHOOTER_RAMP_ACTIVE = 0.33;
    public static double SHOOTER_RAMP_DEACTIVE = 0.0;
    public static double RESTING_SPINDEX_POS = 0.0;

    public static double TRANSFER_POWER = -1.0;

    public double SHOOTER_INTAKE_SPEED = 0.0; // this is the speed where the shooter melonbotic servo intakes the balls

    public static double SHOOTER_INTAKING_SPEED = 1.0;
    public static double SHOOT_ONE_ROTATION = -(2D / 3D) * Math.PI;

    public double intakeWallPosition1 = INTAKE_WALL_1_UP;
    public double intakeWallPosition2 = INTAKE_WALL_2_UP;
    public double shooterRampPosition = SHOOTER_RAMP_DEACTIVE;

    public static double DIDDY_POLE_ACTIVE = 0.6;

    public static double DIDDY_POLE_DEACITVE = 0.98;

    public double diddyPos = DIDDY_POLE_DEACITVE;

    public double spindexerPower = 0.0;
    public TerrorColorSensor[] sensors;

    public boolean transferActive = false;

    public static double LEFT_POSITION = (4D / 3D) * Math.PI;
    public static double RIGHT_POSITION = (2D / 3D) * Math.PI;
    public static double BACK_POSITION = 0.0;
    public static double READY_POSITION = (1D / 6D) * Math.PI; //position for the first ball as the ramp goes down
    double[] yawOffsets = {0, (2.0 / 3) * Math.PI, -((2.0 / 3) * Math.PI)}; // todo: this is currently duplicate, make it so it just uses the above 3 variables

    public static PidfController.PidfCoefficients turningPidCoefficients =
            new PidfController.PidfCoefficients(0.011, 0, 0.00001, 0, 0);
    public static double yawPidTolerance = Math.toRadians(10); // radians
    private boolean pidEnabled = true;
    public final PidfController yawPid = new PidfController(turningPidCoefficients);

    public double desiredAngle;

    public SpindexerSubsystem(RobotHardware hardware, Robot robot) {
        this.robot = robot;
        this.hardware = hardware;
        this.sensors = new TerrorColorSensor[]{hardware.rightSensor, hardware.topSensor, hardware.leftSensor};

        this.desiredAngle = 0;
        this.yawPid.setTolerance(radiansToTicks(yawPidTolerance));
        this.yawPid.setTargetPosition(0.0);
    }

    private static double ticksToRadians(double ticks) {
        return (ticks / TICKS_PER_REVOLUTION) * 2.0 * Math.PI;
    }

    private static double radiansToTicks(double radians) {
        return (radians / (2.0 * Math.PI)) * TICKS_PER_REVOLUTION;
    }

    public double getTargetYaw() {
        return ticksToRadians(this.yawPid.getTargetPosition());
    }

    public boolean atTargetYaw() {
        // TODO: potentially beware of angle wrapping here
        return this.yawPid.atTargetPosition(getPositionTicks());
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
    }

    /**
     * Increases the desired angle by a certain amount.
     * @param angle The angle to add, in radians.
     */
    public void rotate(double angle) {
        this.desiredAngle += angle;
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

    public boolean getLimitSwitchState() {
        return !this.hardware.spindexerLimitSwitch.getState();
    }

    public void setPidEnabled(boolean enabled) {
        spindexerPower = 0.0;
        pidEnabled = enabled;
    }

    public void updateSpindexer() {
        this.yawPid.setTargetPosition(radiansToTicks(desiredAngle));
//        if(hardware.spindexerEncoder.getCurrentPosition())
        if (pidEnabled) {
            this.spindexerPower = yawPid.calculatePower(getPositionTicks(), 0);
        }

        // setting pid power into the spindexer
    }

    public char[] getBallPositions() {
        return new char[]{hardware.topSensor.getGreenOrPurple(), hardware.rightSensor.getGreenOrPurple(), hardware.leftSensor.getGreenOrPurple()};
    }

    public void selectColor(char color) {
        int nearestIndex = new String(getBallPositions()).indexOf(color);
        if (nearestIndex == -1) {
            //TODO: add error handling here some telemetry message abt not having balls or smth
            return;
        }
        goToAngle360(this.yawPid.getTargetPosition() + yawOffsets[nearestIndex]);
    }

    public double getPositionTicks() {
        return hardware.spindexerEncoder.getCurrentPosition() - this.homedSpindexerOffset - SPINDEXER_OFFSET_2;
    }

    public double getPosition() {
        return ticksToRadians(getPositionTicks());
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

    public void setHomedSpindexerOffset(double offset) {
        this.homedSpindexerOffset = offset;
    }

    public void sortBalls() {
        double startPos = this.getPositionTicks();
        int fullCount = 0;
        double greenPos = 0.0;
        int greenCount = 0;
        int purpleCount = 0;
        for (TerrorColorSensor sensor : this.sensors) {
            if (sensor.getGreenOrPurple() != 'N') {
                fullCount += 1;
                if (sensor.getGreenOrPurple() == 'G') {
                    greenCount += 1;
                    if (sensor.position.equals(TerrorColorSensor.side.LEFT)) {
                        greenPos = LEFT_POSITION;
                    } else if (sensor.position.equals(TerrorColorSensor.side.RIGHT)) {
                        greenPos = RIGHT_POSITION;
                    } else {
                        greenPos = BACK_POSITION;
                    }

                } else {
                    purpleCount += 1;
                }

            }
        }
        if (purpleCount == 2 && greenCount == 1) {
            if (robot.camera.gameGlyph == CameraSubsystem.GLYPH.GPP) {
                double normalizedError = MathUtils.normalizeRadians((READY_POSITION - greenPos), true);
                if (normalizedError >= 0.1) {
                    normalizedError = -((2 * Math.PI) - normalizedError);
                }
                this.goToAngle360(startPos + normalizedError);

            } else if (robot.camera.gameGlyph == CameraSubsystem.GLYPH.PGP) {
                double normalizedError = MathUtils.normalizeRadians(((READY_POSITION - ((2D / 3D) * Math.PI)) - greenPos), true);
                if (normalizedError >= 0.1) {
                    normalizedError = -((2 * Math.PI) - normalizedError);
                }
                this.goToAngle360(startPos + normalizedError);

            } else {
                double normalizedError = MathUtils.normalizeRadians(((READY_POSITION - ((4D / 3D) * Math.PI)) - greenPos), true);
                if (normalizedError >= 0.1) {
                    normalizedError = -((2 * Math.PI) - normalizedError);
                }
                this.goToAngle360(startPos + normalizedError);
            }
        } else {
            this.goToAngle360(READY_POSITION);
        }

    }

    public void setSpindexerPower(double power) {
        this.spindexerPower = power;
    }

    @Override
    public void periodic() {
        //to the spindexers of Australia: Robot.camera.getBalls();
        //G:green P:purple N:none
        //0:top 1:right 2:left
        //returns char[]

//        this.hardware.spindexerIntakeRampServo1.setPosition(this.intakeRampPosition1);
//        this.hardware.spindexerIntakeRampServo2.setPosition(this.intakeRampPosition2);
//        this.hardware.spindexerTransferRampServo.setPosition(this.shooterRampPosition);
//        this.hardware.spindexerDiddyServo.setPosition(this.wallPosition);

        this.updateSpindexer();
        this.hardware.spindexerRotate.setPower(this.spindexerPower);
        this.hardware.spindexerIntakeWallServo1.setPosition(intakeWallPosition1);
        this.hardware.spindexerIntakeWallServo2.setPosition(intakeWallPosition2);
        this.hardware.spindexerDiddyServo.setPosition(diddyPos);
        this.hardware.spindexerTransferRampServo.setPosition(shooterRampPosition);
    }
}
