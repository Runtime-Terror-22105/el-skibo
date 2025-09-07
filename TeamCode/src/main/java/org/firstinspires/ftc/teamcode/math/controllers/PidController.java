package org.firstinspires.ftc.teamcode.math.controllers;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.math.Angle;

public class PidController {
    /**
     * Constant to prevent integral windup
     */
    private static final double MAX_INTEGRAL = 1e15;

    // pid constants
    private PidCoefficients pidCoefficients;
    //region pid temp vars
    private double integralSum;
    private double lastError;
    private double lastTimeStamp;
    //endregion

    //region public variables
    private double targetPosition = 0;
    private double tolerance = 10;

    //endregion
    public PidController(PidCoefficients pidCoefficients) {
        this.pidCoefficients = pidCoefficients;
        _resetTempVars();
    }

    public void setPidCoefficients(PidCoefficients pidCoefficients) {
        this.pidCoefficients = pidCoefficients;
    }

    public double getTolerance() {
        return this.tolerance;
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public double calculateError(double encoderPosition) {
        return this.targetPosition - encoderPosition;
    }

    /**
     * NOTE: You must run this function each loop iteration. It will do the PID stuff to calculate
     * the power to be used.
     * @param currentPosition The current position
     */
    public double calculatePower(double currentPosition) {
        return calculatePower(currentPosition, false);
    }

    /**
     * NOTE: You must run this function each loop iteration. It will do the PID stuff to calculate
     * the power to be used.
     * @param currentPosition The current position
     * @param angleWrapError Whether or not to anglewrap the error (-180 to 180)
     */
    public double calculatePower(double currentPosition, boolean angleWrapError) {
        double error = calculateError(currentPosition);
        if (angleWrapError) {
            error = Angle.angleWrap(error);
        }

        double timestamp = (double) System.nanoTime() / 1E9;
        if (lastTimeStamp == 0) lastTimeStamp = timestamp;
        double period = Math.max(timestamp - lastTimeStamp, 1E-5);
        lastTimeStamp = timestamp;

        double derivative = (error - lastError) / period;
        this.integralSum = Math.max(Math.min(integralSum + (error * period), MAX_INTEGRAL), -MAX_INTEGRAL);

        double power = (pidCoefficients.Kp * error)
                + (pidCoefficients.Kd * derivative)
                + (pidCoefficients.Ki * integralSum);
        this.lastError = error;

        if (Math.abs(error) > tolerance) {
            power += power >= 0 ? pidCoefficients.Kstatic : -pidCoefficients.Kstatic;
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

    public boolean atTargetPosition(double currentPosition) {
        return Math.abs(calculateError(currentPosition)) <= this.tolerance;
    }

    /**
     * Gets the current target position of the PID.
     * @return targetPosition - The target position.
     */
    public double getTargetPosition() {
        return this.targetPosition;
    }

    public void _resetTempVars() {
        this.integralSum = 0;
        this.lastError = 0;
        this.lastTimeStamp = 0;
    }

    public static class PidCoefficients {
        public double Kp;
        public double Ki;
        public double Kd;
        public double Kstatic;

        public PidCoefficients(double Kp, double Ki, double Kd, double Kstatic) {
            this.Kp = Kp;
            this.Ki = Ki;
            this.Kd = Kd;
            this.Kstatic = Kstatic;
        }

        public PidCoefficients(double Kp, double Ki, double Kd) {
            this(Kp, Ki, Kd, 0);
        }
    }
}
