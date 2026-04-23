package org.firstinspires.ftc.teamcode.robot.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.controllers.PidfController;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;
import org.firstinspires.ftc.teamcode.util.ArrayUtil;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.Profiler;

import java.util.Arrays;
import java.util.stream.Collectors;

@Config
public class SpindexerSubsystem extends SubsystemBase {

    public boolean inferMissingColorToSort = true;

    private final RobotHardware hardware;
    private final Robot robot;

    public static boolean debug = false;
    public static boolean telemetry = true;

//    public static int NUM_BALLS = 0;

    public static double TIME_TO_PUT_DOWN_WALLS_AFTER_SPINDEX = 150; // milliseconds

    public static double MANUAL_SPINDEXER_DEGREE_CHANGE = 0.5;

    public static double READY_POSITION = 0.52359877559829887307710723054658;

    public static double INTAKE_WALL_LEFT_DOWN = 0.05;
    public static double INTAKE_WALL_LEFT_UP = 0.9;
    public static double INTAKE_WALL_RIGHT_DOWN = 0.85;
    public static double INTAKE_WALL_RIGHT_UP = 0;

    public static double SHOOTER_RAMP_ACTIVE = 0.55;
    public static double SHOOTER_RAMP_DEACTIVE = 0.8;

    public static double MAX_POWER_SORTING = 0.5;
    public boolean useMaxPower = false;

    private enum WallState {
        UP,
        DOWN
    }
    private WallState wallState = WallState.UP;

    private enum RampState {
        ACTIVE,
        DEACTIVE
    }
    private RampState shooterRampPosition = RampState.DEACTIVE;

    public double spindexerPower = 0.0;


//    public static double READY_POSITION = 0.52359877559829887307710723054658; //position for the first ball as the ramp goes down
    double[] yawOffsets = {0, (2.0 / 3) * Math.PI, -((2.0 / 3) * Math.PI)};

    public static PidfController.PidfCoefficients turningPidCoefficientsCcw =
            new PidfController.PidfCoefficients(0.23, 0, 0.0075, 0, 0.12);
    public static PidfController.PidfCoefficients turningPidCoefficientsCw =
            new PidfController.PidfCoefficients(0.23, 0, 0.009, 0, 0.1);
    public static double yawPidTolerance = 0.05; // radians, used for kstatic
    public static double CHECKING_TOLERANCE = 0.09; // radians, only for checking if at target, not for PID tolerance
    private boolean pidEnabled = true;
    public final PidfController yawPid = new PidfController(turningPidCoefficientsCcw);

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

    public void setTolerance(double tolerance) {
        this.yawPid.setTolerance(tolerance);
    }

    public double getTargetYaw() {
        return desiredAngle;
    }

    public boolean atTargetYaw() {
        this.updateSpindexerPid();
        return this.yawPid.atTargetPositionWithTolerance(getPositionRaw(), CHECKING_TOLERANCE, true);
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
        return this.wallState == WallState.DOWN;
    }

    public void setWallDown() {
        this.goingToMoveWallsDownButHaventMovedThemDownYet = true;
        this.goingToMoveWallsDownTimerStarted = false;
    }

    public void setWallUp() {
        this.goingToMoveWallsDownButHaventMovedThemDownYet = false;
        this.goingToMoveWallsDownTimerStarted = false;
        this.wallState = WallState.UP;
    }

    public void enableRamp() {
        shooterRampPosition = RampState.ACTIVE;
    }

    public void disableRamp() {
        shooterRampPosition = RampState.DEACTIVE;
    }

    public BallColor[] getBallPositions() {
        return hardware.colorSensors.readBallColors();
    }

    public double selectColor(BallColor color) {
        int nearestIndex = ArrayUtil.indexOf(getBallPositions(), color);
        return yawOffsets[nearestIndex];
    }

    public void bruteSort() {

        BallColor[] balls = getBallPositions();
        BallColor[] glyphArr = robot.camera.getGlyphCharArray();

        if (glyphArr == null){
            return;
        }


        String letters = Arrays.stream(balls)
                .map(b -> String.valueOf(b.toChar()))
                .collect(Collectors.joining());

        robot.telemetry.addData("i saw ",letters);

        String motif = Arrays.stream(glyphArr)
                .map(b -> String.valueOf(b.toChar()))
                .collect(Collectors.joining());

        robot.telemetry.addData("with motif ",motif);

        if(letters.equals("PPP") || letters.equals("GGG"))
        {
            //idt this works long term maybe it does idk just make sure to keep track of this
            return;
        }

        char[] ideal = motif.toCharArray();
        char[] base = letters.toCharArray();

        int bestScore = -1;
        int bestRotation = 0;

        for (int r = 0; r < 3; r++) {
            int score = 0;

            if (base[r % 3] == ideal[(robot.camera.getBallsSeen()) % 3]) score++;
            if (base[(r + 1) % 3] == ideal[(robot.camera.getBallsSeen() + 1) % 3]) score++;
            if (base[(r + 2) % 3] == ideal[(robot.camera.getBallsSeen() + 2) % 3]) score++;

            if (score > bestScore) {
                bestScore = score;
                bestRotation = r;
            }
        }

        String result = "" + base[bestRotation % 3]
                + base[(bestRotation + 1) % 3]
                + base[(bestRotation + 2) % 3];

        robot.telemetry.addData("attempting to do pattern",result);

        String ccw = letters.substring(1) + letters.charAt(0);
        String cw  = letters.charAt(2) + letters.substring(0, 2);

        double rotateAmount = Math.toRadians(120);

        /*

          //rule
        //+2: rotate forward
        //+1 rotate backward
        //-1 rotate forward
        //-2 rotate backward

        switch(ArrayUtil.indexOf(balls, BallColor.GREEN) - ArrayUtil.indexOf(glyphArr, BallColor.GREEN))
        {
            case 0:
                this.rotate(READY_POSITION);
                break;

            case 2:
            case -1:
                this.rotate(rotateAmount + SpindexerSubsystem.READY_POSITION);
                break;

            case 1:
            case -2:
                this.rotate(-rotateAmount + SpindexerSubsystem.READY_POSITION);
         */

        if (result.equals(ccw))
        {
            this.rotate(-rotateAmount + SpindexerSubsystem.READY_POSITION);
            return;
        }
        if (result.equals(cw))
        {
            this.rotate(rotateAmount + SpindexerSubsystem.READY_POSITION);
            return;
        }
        //no rotation
        this.rotate(READY_POSITION);
        return;
    }

    /**
     * <p>PLEASE DO NOT USE THIS WILLY NILLY!!!
     * you gotta use the command because it only can do the senses when the balls get aligned</p>
     *
     * Newsort returns a boolean indicating whether or not the motif was detected.
     * If it was detected, it returns true (99.99% of cases)
//     * */
//    public boolean newSort()
//    {
//        BallColor[] balls = getBallPositions();
//        BallColor[] glyphArr;
//        if(robot.camera.getRampCVEnabled())
//        {
//            glyphArr = robot.camera.getGlyphCharArray(
//                    robot.camera.getIdealBallPattern()
//            );
//        }
//        else
//        {
//            glyphArr = robot.camera.getGlyphCharArray();
//        }
//
//        if (glyphArr == null) { return false; }
//
//        Log.d("SpindexerSubsystem", "sorting with the following balls: " + balls[0] + ","+balls[1]+","+balls[2]);
//
//        // todo: test if this works and isn't sus
//        // we assume there are always 2 purples and 1 green, so even if one is none, we can infer what it is
//        // if there's two purples and one none, we can infer the none is green
//        // if there's one purple and one green, we can infer the none is purple
//        if (inferMissingColorToSort && ArrayUtil.count(balls, BallColor.NONE) == 1) {
//            if (ArrayUtil.count(balls, BallColor.PURPLE) == 2) {
//                int noneIndex = ArrayUtil.indexOf(balls, BallColor.NONE);
//                balls[noneIndex] = BallColor.GREEN;
//            } else if (ArrayUtil.count(balls, BallColor.PURPLE) == 1 &&
//                    ArrayUtil.count(balls, BallColor.GREEN) == 1) {
//                int noneIndex = ArrayUtil.indexOf(balls, BallColor.NONE);
//                balls[noneIndex] = BallColor.PURPLE;
//            }
//        }
//
//        if (ArrayUtil.contains(balls, BallColor.NONE) ||
//                !ArrayUtil.contains(balls, BallColor.GREEN) ||
//                !ArrayUtil.contains(balls, BallColor.PURPLE))
//        {
//            Log.d("SpindexerSubsystem", "not enough balls to run logic");
//            return true;
//        }
//
//        if(ArrayUtil.count(balls, BallColor.GREEN) != 1)
//        {
//            Log.d("SpindexerSubsystem", "too many greens to run logic");
//            return true;
//        }
//
//        //# = ours-game
//
//        //OURS: PPG
//        //GAME: GPP
//        //+2 rotate forward
//
//        //OURS: PPG
//        //GAME: PGP
//        //+1 rotate backward
//
//        //OURS GPP
//        //GAME: PGP
//        //-1 rotate forward
//
//        //OURS GPP
//        //GAME PPG
//        //-2 rotate backward
//
//
//        //rule
//        //+2: rotate forward
//        //+1 rotate backward
//        //-1 rotate forward
//        //-2 rotate backward
//
//        double rotateAmount = Math.toRadians(120);
//
//        switch(ArrayUtil.indexOf(balls, BallColor.GREEN) - ArrayUtil.indexOf(glyphArr, BallColor.GREEN))
//        {
//            case 0:
//                this.rotate(READY_POSITION);
//                break;
//
//            case 2:
//            case -1:
//                this.rotate(rotateAmount + SpindexerSubsystem.READY_POSITION);
//                break;
//
//            case 1:
//            case -2:
//                this.rotate(-rotateAmount + SpindexerSubsystem.READY_POSITION);
//                break;
//        }
//        return true;
//    }

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

    private void updateSpindexerPid() {
        SpindexerEncoderLUT.SpindexLookupValue desAngle = this.angleLUT.get(desiredAngle);
        if (telemetry) Robot.debugTelemetry.addData("Spindexer Corrected Target (deg)", Math.toDegrees(Angle.angleWrap(desAngle.correctedAngleRad)));

        // Positive wrapped error means CCW, negative means CW.
        double wrappedError = Angle.angleWrap(desAngle.correctedAngleRad - getPositionRaw());
        boolean turningCcw = wrappedError >= 0;
        this.yawPid.setPidfCoefficients(turningCcw ? turningPidCoefficientsCcw : turningPidCoefficientsCw);
        Robot.debugTelemetry.addData("Spindexer PID Mode", turningCcw ? "CCW" : "CW");
        if (debug) Log.d("SpindexerSubsystem", "setting PID coefficients to " + (turningCcw ? turningPidCoefficientsCcw : turningPidCoefficientsCw));

        this.yawPid.setTolerance(yawPidTolerance);
        this.yawPid.setTargetPosition(Angle.angleWrap(desAngle.correctedAngleRad));
    }

    public void updateSpindexer() {
        // setTargetPosition as 0.0 is intentional since PID does not account for angle wrapping, so
        // we calculate error ourselves and feed into PID.
        this.updateSpindexerPid();

        if (pidEnabled) {
//            double error = MathFunctions.getSmallestAngleDifference(desiredAngle, getPosition()) * MathFunctions.getTurnDirection(getPosition(), desiredAngle);
//            this.spindexerPower = Math.copySign(Math.sqrt(Math.abs(tmp)), tmp);
            this.spindexerPower = hardware.getVoltageScale() * yawPid.calculatePower(getPositionRaw(), 0, true);
        }

        if (robot.getAutoSort() && Math.abs(this.spindexerPower) > 0.3){
            this.spindexerPower = Math.copySign(MAX_POWER_SORTING, this.spindexerPower);
        }
    }



    @Override
    public void periodic() {
        try (Profiler.Scope p = Profiler.enter("SpindexerSubsystem")) {

            if (robot.robotState.equals(RobotState.HANGING)) {
                hardware.spindexer.setPower(0);
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


            this.hardware.spindexer.setPower(clampedPower);
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
                        wallState = WallState.DOWN;
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

            this.hardware.wallServoLeft.setPosition(isWallDown() ? INTAKE_WALL_LEFT_DOWN : INTAKE_WALL_LEFT_UP);
            this.hardware.wallServoRight.setPosition(isWallDown() ? INTAKE_WALL_RIGHT_DOWN : INTAKE_WALL_RIGHT_UP);
            this.hardware.transferRampServo.setPosition(shooterRampPosition.equals(RampState.ACTIVE) ? SHOOTER_RAMP_ACTIVE : SHOOTER_RAMP_DEACTIVE);


            if (telemetry) Robot.debugTelemetry.addData("Spindexer Power", clampedPower);
//        Robot.debugTelemetry.addData("Intake Current", this.hardware.intake.getCurrent(CurrentUnit.AMPS));
//        Robot.debugTelemetry.addData("Spindexer Current", this.hardware.spindexerRotate.getCurrent(CurrentUnit.AMPS));
            if (telemetry) Robot.debugTelemetry.addData("Spindexer Position (deg)", Math.toDegrees(Angle.angleWrap(getPosition())));
            if (telemetry) Robot.debugTelemetry.addData("Spindexer Target (deg)", Math.toDegrees(Angle.angleWrap(getTargetYaw())));
        }
    }
}
