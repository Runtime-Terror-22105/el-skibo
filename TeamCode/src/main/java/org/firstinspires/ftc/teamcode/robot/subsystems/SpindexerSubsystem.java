package org.firstinspires.ftc.teamcode.robot.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.controllers.PidfController;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;
import org.firstinspires.ftc.teamcode.robot.subsystems.vision.CameraSubsystem;
import org.firstinspires.ftc.teamcode.util.ArrayUtil;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.Profiler;

@Config
public class SpindexerSubsystem extends SubsystemBase {

    public boolean inferMissingColorToSort = true;

    private final RobotHardware hardware;
    private final Robot robot;

    public static boolean debug = false;
    public static boolean telemetry = true;

    public static double TIME_TO_PUT_DOWN_WALLS_AFTER_SPINDEX = 150; // milliseconds

    public static double MANUAL_SPINDEXER_DEGREE_CHANGE = 0.5;

    public static double READY_POSITION = 0.52359877559829887307710723054658;

    public static double INTAKE_WALL_DOWN = 0.8;
    public static double INTAKE_WALL_UP = 0.1;

    public static double SHOOTER_RAMP_ACTIVE = 0.3;
    public static double SHOOTER_RAMP_DEACTIVE = 0.00;

    public static double MAX_POWER_SORTING = 0.4;
    public boolean useMaxPower = false;

    public double intakeWallPosition = INTAKE_WALL_UP;
    public double shooterRampPosition = SHOOTER_RAMP_DEACTIVE;

    public static double TICKS_PER_REVOLUTION = ((1D + (46D / 11D)) * 28D) * (225D/32D);

    public double spindexerPower = 0.0;

//    public static double READY_POSITION = 0.52359877559829887307710723054658; //position for the first ball as the ramp goes down
    double[] yawOffsets = {0, (2.0 / 3) * Math.PI, -((2.0 / 3) * Math.PI)};

    public static PidfController.PidfCoefficients turningPidCoefficients =
            new PidfController.PidfCoefficients(0.25, 0, 0.013, 0, 0.15);
    public static double yawPidTolerance = Math.toRadians(4); // radians
    private boolean pidEnabled = true;
    public final PidfController yawPid = new PidfController(turningPidCoefficients);

    public double desiredAngle;
    public SpindexerEncoderLUT angleLUT;

    private double homedSpindexerOffset;

    private boolean goingToMoveWallsDownButHaventMovedThemDownYet;
    private ElapsedTime goingToMoveWallsDownTimer = new ElapsedTime();
    private boolean goingToMoveWallsDownTimerStarted = false;
    public boolean overrideMaxPower;

    public SpindexerSubsystem(RobotHardware hardware, Robot robot) {
        this.robot = robot;
        this.hardware = hardware;
        this.homedSpindexerOffset = 0;
//        this.desiredAngle = getPosition();
        this.yawPid.setTolerance(yawPidTolerance);
        this.yawPid.setTargetPosition(0.0);
        this.angleLUT = new SpindexerEncoderLUT(this.robot);
        goToAngle120(0);
    }

    public static double ticksToRadians(double ticks) {
        return (ticks / TICKS_PER_REVOLUTION) * 2.0 * Math.PI;
    }

    private static double radiansToTicks(double radians) {
        return (radians / (2.0 * Math.PI)) * TICKS_PER_REVOLUTION;
    }

    public double getPositionRaw() {
        return hardware.spindexerEncoder.getCurrentPosition();
    }

    public double getPosition() {
        return Angle.angleWrap(hardware.spindexerEncoder.getCurrentPosition());
    }

    // TODO: restore this eventually
//    public double getPosition() {
////        return Angle.angleWrap(hardware.spindexerEncoder.getCurrentPosition());
//    }


    public double getTargetYaw() {
        return desiredAngle;
    }

    public boolean atTargetYaw() {
        // TODO: potentially beware of angle wrapping here
        return this.yawPid.atTargetPosition(getPositionRaw(), true);
    }

    public void setHomedSpindexerOffset(double offset) {
        this.homedSpindexerOffset = offset;
    }
    public double getHomedSpindexerOffset() {
        return this.homedSpindexerOffset;
    }


    public void goToNearestSide()
    {
        goToAngle120(0);
    }

    /**
     * <p>Snaps the spindexer to a desired angle, while doing modulus to avoid unnecessary rotation.</p>
     * <p>Use this when moving to an angle less than 120 degrees away.</p>
     * @param angle The angle to go to, in radians.
     */
    public void goToAngle120(double angle) {
//        this.desiredAngle = angle;
        // TODO restore this function later once spindexer is no longer cooked
        double currAngle = getPositionRaw();
        double bestAngle = currAngle;
        double bestError = Double.POSITIVE_INFINITY;

        double sector = 2 * Math.PI / 3.0; // 120 deg

        // basically just loop through the three sides to see which is optimal
        for (int i = 0; i < 3; i++) {
            double equiv = angle + i * sector; // the equivalent angle in our current area
            double error = Angle.angleWrap(equiv - currAngle);
            if (Math.abs(error) < bestError) {
                bestError = Math.abs(error);
                bestAngle = currAngle + error;
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
        if (debug) Log.d("SpindexerSubsystem", "desired angle gotTo360" + this.desiredAngle);
    }

    /**
     * Increases the desired angle by a certain amount.
     * @param angle The angle to add, in radians.
     */
    public void rotate(double angle) {
        this.desiredAngle += angle;
    }

    public boolean isWallDown() {
        return this.intakeWallPosition == INTAKE_WALL_DOWN;
    }

    public void setWallDown() {
        this.goingToMoveWallsDownButHaventMovedThemDownYet = true;
        this.goingToMoveWallsDownTimerStarted = false;
    }

    public void setWallUp() {
        this.goingToMoveWallsDownButHaventMovedThemDownYet = false;
        this.goingToMoveWallsDownTimerStarted = false;
        this.intakeWallPosition = INTAKE_WALL_UP;
    }

    public void enableRamp() {
        shooterRampPosition = SHOOTER_RAMP_ACTIVE;
    }

    public void disableRamp() {
        shooterRampPosition = SHOOTER_RAMP_DEACTIVE;
    }

    public BallColor[] getBallPositions() {
        return hardware.colorSensors.readBallColors();
    }

    public double selectColor(BallColor color) {
        int nearestIndex = ArrayUtil.indexOf(getBallPositions(), color);
        return yawOffsets[nearestIndex];
    }

    /**
     * <p>PLEASE DO NOT USE THIS WILLY NILLY!!!
     * you gotta use the command because it only can do the senses when the balls get aligned</p>
     *
     * Newsort returns a boolean indicating whether or not the motif was detected.
     * If it was detected, it returns true (99.99% of cases)
     * */
    public boolean newSort()
    {
        BallColor[] balls = getBallPositions();
        Log.d("SpindexerSubsystem", "balls" + balls[0].toChar() + balls[1].toChar() +balls[2].toChar());
        BallColor[] glyphArr = robot.camera.getGlyphCharArray();
        if (glyphArr == null) { return false; }

        Log.d("SpindexerSubsystem", "sorting with the following balls: " + balls[0] + ","+balls[1]+","+balls[2]);

        // todo: test if this works and isn't sus
        // we assume there are always 2 purples and 1 green, so even if one is none, we can infer what it is
        // if there's two purples and one none, we can infer the none is green
        // if there's one purple and one green, we can infer the none is purple
        if (inferMissingColorToSort && ArrayUtil.count(balls, BallColor.NONE) == 1) {
            if (ArrayUtil.count(balls, BallColor.PURPLE) == 2) {
                int noneIndex = ArrayUtil.indexOf(balls, BallColor.NONE);
                balls[noneIndex] = BallColor.GREEN;
            } else if (ArrayUtil.count(balls, BallColor.PURPLE) == 1 &&
                    ArrayUtil.count(balls, BallColor.GREEN) == 1) {
                int noneIndex = ArrayUtil.indexOf(balls, BallColor.NONE);
                balls[noneIndex] = BallColor.PURPLE;
            }
        }

        if (ArrayUtil.contains(balls, BallColor.NONE) ||
                !ArrayUtil.contains(balls, BallColor.GREEN) ||
                !ArrayUtil.contains(balls, BallColor.PURPLE))
        {
            Log.d("SpindexerSubsystem", "not enough balls to run logic");
            return true;
        }

        if(ArrayUtil.count(balls, BallColor.GREEN) != 1)
        {
            Log.d("SpindexerSubsystem", "too many greens to run logic");
            return true;
        }

        //# = ours-game

        //OURS: PPG
        //GAME: GPP
        //+2 rotate forward

        //OURS: PPG
        //GAME: PGP
        //+1 rotate backward

        //OURS GPP
        //GAME: PGP
        //-1 rotate forward

        //OURS GPP
        //GAME PPG
        //-2 rotate backward


        //rule
        //+2: rotate forward
        //+1 rotate backward
        //-1 rotate forward
        //-2 rotate backward

        double rotateAmount = Math.toRadians(120);

        switch(ArrayUtil.indexOf(balls, BallColor.GREEN) - ArrayUtil.indexOf(glyphArr, BallColor.GREEN))
        {
            case 0:
                break;

            case 2:

            case -1:

                this.rotate(rotateAmount);
                break;

            case 1:

            case -2:
                this.rotate(-rotateAmount);
                break;
        }
        return true;
    }

    public void sortBalls() {
        Log.d("SpindexerSubsystem", "des ang before sort "+ this.desiredAngle);
        Log.d("SpindexerSubsystem", "des ang before sort deg "+ this.desiredAngle * (180D/Math.PI));
        this.goToAngle120(0);
        Log.d("SpindexerSubsystem", "des ang aft 0 "+ this.desiredAngle);
        int fullCount = 0;
        double greenPos = 0.0;
        int greenCount = 0;
        int purpleCount = 0;
        for (BallColor ball : this.getBallPositions()) {
            Log.d("ball-thing", String.valueOf(ball.toChar()));
            if (!BallColor.NONE.equals(ball)) {
                fullCount += 1;
                if (!BallColor.GREEN.equals(ball)) {
                    greenCount += 1;
                    greenPos = this.selectColor(BallColor.GREEN);
                } else {
                    purpleCount += 1;
                }

            }
        }
        Log.d("SpindexerSubsystem", "purple count" + purpleCount);
        Log.d("SpindexerSubsystem", "green count" + greenCount);
        Log.d("SpindexerSubsystem", "full count" + fullCount);
        Log.d("SpindexerSubsystem", "greenPos" + greenPos);

        if (purpleCount == 2 && greenCount == 1) {
            if (robot.camera.gameGlyph == CameraSubsystem.GLYPH.GPP) {
                double normalizedError = MathUtils.normalizeRadians(-greenPos, false);
                Log.d("SpindexerSubsystem", "glyph gpp normalized error" + normalizedError);
                this.rotate(normalizedError);

            } else if (robot.camera.gameGlyph == CameraSubsystem.GLYPH.PGP) {
                double normalizedError = MathUtils.normalizeRadians((((2D / 3D) * Math.PI)) - greenPos, false);
                Log.d("SpindexerSubsystem", "glyph pgp normalized error" + normalizedError);
                this.rotate(normalizedError);

            } else {
                double normalizedError = MathUtils.normalizeRadians((((4D / 3D) * Math.PI)) - greenPos, false);
                Log.d("SpindexerSubsystem", "glyph ppg normalized error" + normalizedError);
                this.rotate(normalizedError);
            }
        }
        Log.d("SpindexerSubsystem", "des ang after sort"+this.desiredAngle);

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
        overrideMaxPower = false;
    }

    public void updateSpindexer() {
        // setTargetPosition as 0.0 is intentional since PID does not account for angle wrapping, so
        // we calculate error ourselves and feed into PID.
        SpindexerEncoderLUT.SpindexLookupValue desAngle = this.angleLUT.get(desiredAngle);
        if (telemetry) Robot.debugTelemetry.addData("Spindexer Corrected Target (deg)", Math.toDegrees(Angle.angleWrap(desAngle.correctedAngleRad)));

        this.yawPid.setTolerance(yawPidTolerance);
        this.yawPid.setTargetPosition(desAngle.correctedAngleRad);
        if (pidEnabled) {
//            double error = MathFunctions.getSmallestAngleDifference(desiredAngle, getPosition()) * MathFunctions.getTurnDirection(getPosition(), desiredAngle);
            this.spindexerPower = yawPid.calculatePower(getPositionRaw(), 0, true);
        }
    }



    @Override
    public void periodic() {
        try (Profiler.Scope p = Profiler.enter("SpindexerSubsystem")) {
            // ensure we update the PID coefficients in case they were changed in dashboard
            this.yawPid.setPidfCoefficients(turningPidCoefficients);

            if (robot.robotState.equals(RobotState.HANGING_90) || robot.robotState.equals(RobotState.HANGING_FINAL)) {
                hardware.spindexerRotate.setPower(0);
                return;
            }

            this.updateSpindexer();

            if (debug) Log.i("SpindexerSubsystem", "initial voltage: " + hardware.initialVoltage);

            double clampedPower;
            if (useMaxPower && !overrideMaxPower){
                 clampedPower = Math.max(-MAX_POWER_SORTING, Math.min(MAX_POWER_SORTING, spindexerPower));
            }
            else{
                clampedPower = spindexerPower;
            }


            this.hardware.spindexerRotate.setPower(clampedPower);
    //        this.hardware.spindexerRotate.setPower(pidEnabled ? spindexerPower : clampedPower);

            // basically this just makes it so that the walls go down at the right time after the
            // pid reaches the target so we never have the walls go down at the wrong spot and jam the spindexer
            if (goingToMoveWallsDownButHaventMovedThemDownYet && // if we want to set walls down
                    desiredAngle % (2 * Math.PI / 3) < Math.toRadians(2) && // and we are setting the angle to a flat side (a multiple of 120 degrees)
                    pidEnabled
            ) {
                if (atTargetYaw()) {
                    if (goingToMoveWallsDownTimerStarted &&
                            goingToMoveWallsDownTimer.milliseconds() > TIME_TO_PUT_DOWN_WALLS_AFTER_SPINDEX) {
                        intakeWallPosition = INTAKE_WALL_DOWN;
                        goingToMoveWallsDownButHaventMovedThemDownYet = false;
                    } else if (!goingToMoveWallsDownTimerStarted) {
                        goingToMoveWallsDownButHaventMovedThemDownYet = true;
                        goingToMoveWallsDownTimerStarted = true;
                        goingToMoveWallsDownTimer.reset();
                    }
                } else {
                    goingToMoveWallsDownButHaventMovedThemDownYet = true;
                    goingToMoveWallsDownTimerStarted = false;
                }
            }

            this.hardware.spindexerIntakeWallServo.setPosition(intakeWallPosition);
            this.hardware.spindexerTransferRampServo.setPosition(shooterRampPosition);


            if (telemetry) Robot.debugTelemetry.addData("Spindexer Power", clampedPower);
//        Robot.debugTelemetry.addData("Intake Current", this.hardware.intake.getCurrent(CurrentUnit.AMPS));
//        Robot.debugTelemetry.addData("Spindexer Current", this.hardware.spindexerRotate.getCurrent(CurrentUnit.AMPS));
            if (telemetry) Robot.debugTelemetry.addData("Spindexer Position (deg)", Math.toDegrees(Angle.angleWrap(getPosition())));
            if (telemetry) Robot.debugTelemetry.addData("Spindexer Target (deg)", Math.toDegrees(Angle.angleWrap(getTargetYaw())));
        }
    }
}
