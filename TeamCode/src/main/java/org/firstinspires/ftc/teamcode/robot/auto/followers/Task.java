package org.firstinspires.ftc.teamcode.robot.auto.followers;

import org.firstinspires.ftc.teamcode.math.Pose2d;

public interface Task {
    enum Type {
        DRIVING,
        ACTION,
        BG_ACTION,
        FINISH_ACTIONS,
        SLEEP
    }

    interface Context {
        void startTimer();

        /**
         * Get the amount of time since the task started (ms).
         * @return The amount of time since the task started, in milliseconds.
         */
        double getTime();

        Pose2d getCurrentPos();

        void setCurrentPos(Pose2d currentPos);
    }

    boolean execute(Context context);

    void kill(Context context);

    /**
     * Whether or not the task has passed its maximum time limit and has stalled.
     * @return A boolean, if true, the task should be killed, if not it should keep going
     */
    boolean taskHasStalled();

    String getName();

    Type getTaskType();

    Context getContext();

}
