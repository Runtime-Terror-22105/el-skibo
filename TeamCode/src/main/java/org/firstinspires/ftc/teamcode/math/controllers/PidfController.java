package org.firstinspires.ftc.teamcode.math.controllers;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.math.Angle;

public class PidfController {
    private static final double MAX_INTEGRAL = 1e15; // random constant to prevent integral windup, will adjust later

    // pid constants
    private PidfCoefficients pidfCoefficients;
    //region pid temp vars
    private double integralSum;
    private double lastError;
    private double lastTimeStamp;
    //endregion

    //region public variables
    private double targetPosition = 0;
    private double tolerance = 0;
    private double currentPosition;

    //endregion
    public PidfController(PidfCoefficients pidfCoefficients) {
        this.pidfCoefficients = pidfCoefficients;
        _resetTempVars();
    }

    public void setPidfCoefficients(PidfCoefficients coefficients) {
        this.pidfCoefficients = coefficients;
    }

    public double getTolerance() {
        return this.tolerance;
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public double calculatePower(double currentPosition, double feedforwardReference, boolean angleWrapError) {
        if (Double.isNaN(this.targetPosition) || Double.isNaN(currentPosition)) {
            return 0;
        }

        this.currentPosition = currentPosition;

        double error = calculateError(currentPosition, angleWrapError);
        return this.calculatePower(error, feedforwardReference);
    }

    /**
     * NOTE: You must run this function each loop iteration. It will do the PID stuff to calculate
     * the power to be used.
     */
    public double calculatePower(double error, double feedforwardReference) {
        double timestamp = (double) System.nanoTime() / 1E9;
        if (lastTimeStamp == 0) lastTimeStamp = timestamp;
        double period = Math.max(timestamp - lastTimeStamp, 1E-5);
        lastTimeStamp = timestamp;

        double derivative = (error - lastError) / period;
        this.integralSum = Math.max(Math.min(integralSum + (error * period), MAX_INTEGRAL), -MAX_INTEGRAL);

        double power = (pidfCoefficients.Kp * error)
                + (pidfCoefficients.Kd * derivative)
                + (pidfCoefficients.Ki * integralSum)
                + (pidfCoefficients.Kv * feedforwardReference);
        this.lastError = error;

        if (Math.abs(error) > tolerance) {
            power += power >= 0 ? pidfCoefficients.Kstatic : -pidfCoefficients.Kstatic;
        }

        power = Range.clip(power, -1, 1);
        if (Double.isNaN(power)) power = 0;

        return power;
    }

    public double getLastError() {
        return lastError;
    }

    /**
     * Increase/decrease the target position of the PID by some amount of counts. This is a
     * RELATIVE move unlike setTargetPosition().
     * @param moveAmt The amount of clicks to increase/decrease the position by.
     */
    public void move(double moveAmt) {
        double newTargetPos = this.targetPosition + moveAmt;
        this.setTargetPosition(newTargetPos);
    }

    /**
     * Sets the target position of the slides.
     * @param targetPosition The new target position for the slides.
     */
    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
    }
    // endregion
    /**
     * Gets the current target position of the PID.
     * @return targetPosition - The target position.
     */
    public double getTargetPosition() {
        return this.targetPosition;
    }

    private void _resetTempVars() {
        this.integralSum = 0;
        this.lastError = 0;
        this.lastTimeStamp = 0;
    }

    public double calculateError(double encoderPosition) {
        return this.calculateError(encoderPosition, false);
    }

    public double calculateError(double encoderPosition, boolean angleWrapError) {
        double error = this.targetPosition - encoderPosition;
        if (angleWrapError) {
            error = Angle.angleWrap(error);
        }
        return error;
    }

    public boolean atTargetPosition() {
        return atTargetPosition(currentPosition);
    }

    public boolean atTargetPosition(double currentPosition) {
        return atTargetPositionWithTolerance(currentPosition, this.tolerance);
    }

    public boolean atTargetPositionWithTolerance(double currentPosition, double tolerance) {
        return Math.abs(calculateError(currentPosition)) <= tolerance;
    }

    public boolean atTargetPosition(double currentPosition, boolean angleWrapError) {
        return Math.abs(calculateError(currentPosition, angleWrapError)) <= this.tolerance;
    }

    public static class PidfCoefficients {
        public double Kp;
        public double Ki;
        public double Kd;
        public double Kv;
        public double Kstatic;

        public PidfCoefficients(double Kp, double Ki, double Kd, double Kv, double Kstatic) {
            this.Kp = Kp;
            this.Ki = Ki;
            this.Kd = Kd;
            this.Kv = Kv;
            this.Kstatic = Kstatic;
        }

        public PidfCoefficients(double Kp, double Ki, double Kd, double Kv) {
            this(Kp, Ki, Kd, Kv, 0);
        }
    }
}
