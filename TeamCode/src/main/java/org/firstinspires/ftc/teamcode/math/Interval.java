package org.firstinspires.ftc.teamcode.math;

/**
 * A simple class to repeat a certain action after a specified time period.
 */
public class Interval {
    private double pollTime;
    private double lastPollTime;

    /**
     * Constructs an Interval with the specified poll time.
     *
     * @param pollTime the time period in milliseconds after which the action should be repeated
     */
    public Interval(double pollTime) {
        this.pollTime = pollTime;
        this.lastPollTime = System.currentTimeMillis();
    }

    /**
     * Checks if the specified interval time has passed since the last poll.
     *
     * @return true if the interval has passed, false otherwise
     */
    public boolean intervalHasPassed() {
        if (getCurrentTimeMillis() >= (lastPollTime + pollTime)) {
            this.lastPollTime = this.getCurrentTimeMillis();
            return true;
        }
        return false;
    }

    /**
     * Gets the current time in milliseconds.
     *
     * @return the current time in milliseconds
     */
    public double getCurrentTimeMillis() {
        return System.nanoTime() / 1e6;
    }

    /**
     * Gets the last poll time in milliseconds.
     *
     * @return the last time the interval was checked in milliseconds
     */
    public double getLastPollTime() {
        return this.lastPollTime;
    }

    /**
     * Sets a new poll time.
     *
     * @param pollTime the new time period in milliseconds for the interval
     */
    public void setPollTime(double pollTime) {
        this.pollTime = pollTime;
    }
}
