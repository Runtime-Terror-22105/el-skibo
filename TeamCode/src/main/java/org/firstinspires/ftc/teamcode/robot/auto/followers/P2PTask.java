package org.firstinspires.ftc.teamcode.robot.auto.followers;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.drive.Drivetrain;

import java.util.function.Consumer;
import java.util.function.Predicate;

public class P2PTask implements Task {
    public static class Context implements Task.Context {
        private final Drivetrain drivetrain;
        public PidToPoint p2p;
        private final ElapsedTime taskTimer;
        private Pose2d currentPos; // NOT FINAL

        public Context(Drivetrain drivetrain) {
            this.drivetrain = drivetrain;
            this.currentPos = new Pose2d();
            this.p2p = new PidToPoint();
            this.taskTimer = new ElapsedTime();
        }

        public Context copy() {
            return new Context(this.drivetrain);
        }

        /**
         * Start the timer for the task
         */
        public void startTimer() {
            this.taskTimer.reset();
        }

        /**
         * Get the amount of time since the task started (ms).
         * @return The amount of time since the task started, in milliseconds.
         */
        public double getTime() {
            return this.taskTimer.milliseconds();
        }

        public Drivetrain getDrivetrain() {
            return drivetrain;
        }

        public Pose2d getCurrentPos() {
            return currentPos;
        }

        public void setCurrentPos(Pose2d currentPos) {
            this.currentPos = currentPos;
        }

        public void setFollower(PidToPoint p2p) {
            this.p2p = p2p;
        }
    }

    private final Type taskType;
    private final Predicate<Context> task;
    private final Consumer<Context> killAction;
    private Context context;
    private final double timeLimit;
    private final String name;
    private PointInfo pointInfo;

    // The name of a task that must be completed before this is executed
    private String previousTask;

    /**
     * @param name The name of the task
     * @param context The task's context
     * @param task The function for the task to run
     * @param killAction A callback to execute if the task is killed
     * @param taskType The type of task
     * @param timeLimit The time limit, in milliseconds
     */
    public P2PTask(String name,
                   Context context,
                   Predicate<Context> task,
                   Consumer<Context> killAction,
                   Type taskType,
                   double timeLimit) {
        this.name = name;
        this.context = context;
        this.task = task;
        this.taskType = taskType;
        this.timeLimit = timeLimit;
        this.killAction = killAction;
        this.previousTask = null;
    }

    /**
     * @param name The name of the task
     * @param context The task's context
     * @param previousTask The name of the task that must precede this
     * @param task The function for the task to run
     * @param killAction A callback to execute if the task is killed
     * @param taskType The type of task
     * @param timeLimit The time limit, in milliseconds
     */
    public P2PTask(String name,
                   String previousTask,
                   Context context,
                   Predicate<Context> task,
                   Consumer<Context> killAction,
                   Type taskType,
                   double timeLimit) {
        this.name = name;
        this.previousTask = previousTask;
        this.context = context;
        this.task = task;
        this.taskType = taskType;
        this.timeLimit = timeLimit;
        this.killAction = killAction;
    }

    public P2PTask(@NonNull P2PTask task) {
        this.name = task.getName();
        this.pointInfo = task.getPointInfo();
        this.context = task.getContext().copy();
        this.task = task.task;
        this.killAction = task.killAction;
        this.taskType = task.getTaskType();
        this.timeLimit = task.timeLimit;
        this.previousTask = task.previousTask;
    }

    public void setPreviousTask(String previousTask) {
        this.previousTask = previousTask;
    }

    public String getPreviousTask() {
        return this.previousTask;
    }

    public void setPointInfo(PointInfo pointInfo) {
        this.pointInfo = pointInfo;
    }

    public PointInfo getPointInfo() {
        return this.pointInfo;
    }

    @Override
    public boolean execute(Task.Context context) {
        return task.test((P2PTask.Context) context);
    }

    @Override
    public void kill(Task.Context context) {
        killAction.accept((P2PTask.Context) context);
    }

    /**
     * Whether or not the task has passed its maximum time limit and has stalled.
     * @return A boolean, if true, the task should be killed, if not it should keep going
     */
    @Override
    public boolean taskHasStalled() {
        return this.context.getTime() > this.timeLimit;
    }

    @Override
    public String getName() {
        return this.name;
    }

    @Override
    public Type getTaskType() {
        return taskType;
    }

    @Override
    public Context getContext() {
        return context;
    }
}
