package org.firstinspires.ftc.teamcode.math.controllers;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.math.Angle;


public class SwervePidfController {
    private static final double MAX_INTEGRAL = 100000;
    // pidf constants
    private SwervePidfCoefficients pidfCoefficients;

    //region pidf temp vars
    private double integralSum;
    private double lastError;
    private double error;
    private double lastTimeStamp;
    //endregion

    //region public variables
    public double power = 0;
    private double targetPosition = 0;

    public double tolerance;
    //endregion

    public SwervePidfController(@NonNull SwervePidfCoefficients pidfCoefficients) {
        this.pidfCoefficients = pidfCoefficients;
        _resetTempVars();
    }

    /**
     * NOTE: You must run this function each loop iteration. It will do the PID stuff to calculate
     * the power to be used.
     */
    private void calculate(double error, double feedforwardReference) {
        double timestamp = (double) System.nanoTime() / 1E9;
        if (lastTimeStamp == 0) lastTimeStamp = timestamp;
        double period = Math.max(timestamp - lastTimeStamp, 1E-5);
        lastTimeStamp = timestamp;

        double derivative = (error - lastError) / period;

        integralSum = Math.max(Math.min(integralSum + (error * period), MAX_INTEGRAL), -MAX_INTEGRAL);

        double outUnclamped = (pidfCoefficients.Kp * error)
                + (pidfCoefficients.Kd * derivative)
                + (pidfCoefficients.Ki * integralSum)
                + (pidfCoefficients.Kv * feedforwardReference);
        power = Math.max(Math.min(outUnclamped, 1.0), -1.0); // out has to be between -1 and 1
        lastError = error;
    }

    public boolean atTargetPosition() {
        return Math.abs(error) < this.tolerance;
    }

    public void resetPid() {
        _resetTempVars();
        power = 0;
    }

    public void setCoefficients(@NonNull SwervePidfCoefficients coefficients) {
        this.pidfCoefficients = coefficients;
    }

    public void calculatePower(double encoderPosition, double feedforwardReference, double tolerance){
        this.tolerance = tolerance;

        this.error = Angle.angleWrap(this.targetPosition - encoderPosition);
        this.calculate(error, feedforwardReference);

        this.power = Range.clip(power, -1, 1);
        if (Double.isNaN(power)) this.power = 0;

        if (Math.abs(error) > tolerance) {
            if (power >= 0) {
                this.power += pidfCoefficients.Kstatic;
            } else {
                this.power -= pidfCoefficients.Kstatic;
            }
        }
        this.power = Range.clip(power, -1, 1);
    }

    public double getPower() {
        return this.power;
    }

    // region moving

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
        this.error = 0;
        this.lastTimeStamp = 0;
    }

    public static class SwervePidfCoefficients {
        public double Kp;
        public double Ki;
        public double Kd;
        public double Kv;
        public double Kstatic;

        public SwervePidfCoefficients(double Kp, double Ki, double Kd, double Kv, double Kstatic) {
            this.Kp = Kp;
            this.Ki = Ki;
            this.Kd = Kd;
            this.Kv = Kv;
            this.Kstatic = Kstatic;
        }
    }
}
