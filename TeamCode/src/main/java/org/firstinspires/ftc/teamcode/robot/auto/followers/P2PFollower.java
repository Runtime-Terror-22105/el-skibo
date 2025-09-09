package org.firstinspires.ftc.teamcode.robot.auto.followers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.math.Coordinate;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Predicate;
import java.util.function.Supplier;

public class P2PFollower implements TerrorFollower {
//    public static class Builder implements TerrorFollower.Builder {
    public static class Builder {
//        private final Queue<Task> tasks = new LinkedList<>();
        private final List<P2PTask> tasks = new ArrayList<>();
        private final boolean onlyDoPathing;
        private final boolean useVerboseErrors;

        public Builder(@NonNull Robot robot) {
            this(robot, false, true);
        }

        public Builder(@NonNull Robot robot, boolean onlyDoPathing, boolean useVerboseErrors) {
            this.onlyDoPathing = onlyDoPathing;
            this.useVerboseErrors = useVerboseErrors;
        }

        public Builder addPoint(
                Pose2d point,
                Pose2d tolerance,
                double reachedTime,
                double timeLimit
        ) {
            return this.addPoint(
                    "Drive to point (" + point.x + "," + point.y + "," + point.heading + ")",
                    point,
                    tolerance,
                    reachedTime,
                    timeLimit
            );
        }

        /**
         * Adds an instruction to drive to a point at full speed.
         * @param pointName The name of the point
         * @param point The point to drive to.
         * @param tolerance The tolerance for how much error there can be on the x,y,h.
         * @param reachedTime How long the robot needs to stay at its destination.
         * @param timeLimit How long the robot can take to drive (ms) before it is considered to have stalled.
         * @return The builder object, to allow for chaining.
         */
        public Builder addPoint(
                String pointName,
                Pose2d point,
                Pose2d tolerance,
                double reachedTime,
                double timeLimit
        ) {
            return this.addPoint(pointName, point, tolerance, 1.0, reachedTime, timeLimit);
        }

        /**
         * Adds an instruction to drive to a point.
         * @param pointName The name of the point
         * @param point The point to drive to.
         * @param tolerance The tolerance for how much error there can be on the x,y,h.
         * @param reachedTime How long the robot needs to stay at its destination.
         * @param timeLimit How long the robot can take to drive (ms) before it is considered to have stalled.
         * @return The builder object, to allow for chaining.
         */
        public Builder addPoint(
                String pointName,
                Pose2d point,
                Pose2d tolerance,
                double speed,
                double reachedTime,
                double timeLimit
        ) {
            P2PTask.Context context = new P2PTask.Context(Robot.drivetrain);
            P2PTask task = new P2PTask(
                    pointName,
                    context,
                    (P2PTask.Context ctx) -> ctx.p2p.calculate(ctx.getCurrentPos()),
                    (P2PTask.Context ctx) -> ctx.getDrivetrain().move(new Coordinate(0,0),0),
                    P2PTask.Type.DRIVING,
                    timeLimit
            );
            task.setPointInfo(new PointInfo(point, tolerance, speed, reachedTime));
            tasks.add(task);

            return this;
        }

    public Builder addPoint(
            String pointName,
            Pose2d point,
            Pose2d tolerance,
            Predicate<P2PTask.Context> condition,
            double speed,
            double reachedTime,
            double timeLimit
    ) {
        P2PTask.Context context = new P2PTask.Context(Robot.drivetrain);
        P2PTask task = new P2PTask(
                pointName,
                context,
                condition,
                (P2PTask.Context ctx) -> ctx.getDrivetrain().move(new Coordinate(0,0),0),
                P2PTask.Type.DRIVING,
                timeLimit
        );
        task.setPointInfo(new PointInfo(point, tolerance, speed, reachedTime));
        tasks.add(task);

        return this;
    }

        /**
         * Execute some action once.
         * @param taskName The name of the task in logs
         * @param action The function to run.
         * @param timeLimit How long the action can take to run (ms), before it is considered to have stalled.
         * @return The builder object, to allow for chaining.
         */
        public Builder executeActionOnce(String taskName, Consumer<P2PTask.Context> action, double timeLimit) {
            if (onlyDoPathing) {
                return this;
            }

            P2PTask.Context context = new P2PTask.Context(Robot.drivetrain);
            P2PTask task = new P2PTask(
                    taskName,
                    context,
                    (P2PTask.Context ctx) -> {
                        action.accept(ctx);
                        return true;
                    },
                    (P2PTask.Context ctx) -> {},
                    P2PTask.Type.ACTION,
                    timeLimit
            );
            tasks.add(task);

            return this;
        }

        /**
         * Simply keeps the drivetrain active and runs the pre/after loop funcs certain duration of time.
         * @param milliseconds How long to wait for.
         * @return The builder object, to allow for chaining.
         */
        public Builder sleep(int milliseconds) {
            String taskName = "Sleep for " + milliseconds + " ms";

            P2PTask.Context context = new P2PTask.Context(Robot.drivetrain);
            P2PTask task = new P2PTask(
                    taskName,
                    context,
                    (P2PTask.Context ctx) -> false,
                    (P2PTask.Context ctx) -> {},
                    P2PTask.Type.SLEEP,
                    milliseconds
            );
            tasks.add(task);

            return this;
        }

        /**
         * Modifies the last task to occur after some other task.
         * @param taskName The name of the task that this should be after.
         * @return The builder object, to allow for chaining.
         */
        public Builder afterTask(String taskName) {
            P2PTask lastTask = tasks.remove(tasks.size()-1);
            lastTask.setPreviousTask(taskName);
            tasks.add(lastTask);
            return this;
        }

        /**
         * Execute some action repeatedly (async) until some condition is true.
         * @param taskName The name of the task to run.
         * @param condition The condition.
         * @param action The function to run.
         * @param timeLimit How long the action can take to run (ms), before it is considered to have stalled.
         * @return The builder object, to allow for chaining.
         */
        public Builder executeUntilTrue(
                String taskName,
                Predicate<P2PTask.Context> condition,
                Consumer<P2PTask.Context> action,
                double timeLimit
        ) {
            if (onlyDoPathing) {
                return this;
            }

            P2PTask.Context context = new P2PTask.Context(Robot.drivetrain);
            P2PTask task = new P2PTask(
                    taskName,
                    context,
                    (P2PTask.Context ctx) -> {
                        action.accept(ctx);
                        return condition.test(ctx);
                    },
                    (P2PTask.Context ctx) -> {},
                    P2PTask.Type.ACTION,
                    timeLimit
            );
            tasks.add(task);

            return this;
        }

        /**
         * Run some action for some amount of time.
         * @param taskName The name of the task to run.
         * @param action The function to run.
         * @param timeLimit The amount of time, in milliseconds
         * @return The builder object, to allow for chaining.
         */
        public Builder executeForTime(
                String taskName,
                Consumer<P2PTask.Context> action,
                double timeLimit
        ) {
            if (onlyDoPathing) {
                return this;
            }

            P2PTask.Context context = new P2PTask.Context(Robot.drivetrain);
            P2PTask task = new P2PTask(
                    taskName,
                    context,
                    (P2PTask.Context ctx) -> {
                        action.accept(ctx);
                        return false;
                    },
                    (P2PTask.Context ctx) -> {},
                    P2PTask.Type.ACTION,
                    timeLimit
            );
            tasks.add(task);

            return this;
        }

        /**
         * Run some action for some amount of time in the background.
         * @param taskName The name of the task to run.
         * @param action The function to run.
         * @param timeLimit The amount of time, in milliseconds
         * @return The builder object, to allow for chaining.
         */
        public Builder executeInBackground(
                String taskName,
                Consumer<P2PTask.Context> action,
                Consumer<P2PTask.Context> killAction,
                double timeLimit
        ) {
            if (onlyDoPathing) {
                return this;
            }

            P2PTask.Context context = new P2PTask.Context(Robot.drivetrain);
            P2PTask task = new P2PTask(
                    taskName,
                    context,
                    (P2PTask.Context ctx) -> {
                        action.accept(ctx);
                        return false;
                    },
                    killAction,
                    P2PTask.Type.BG_ACTION,
                    timeLimit
            );
            tasks.add(task);

            return this;
        }

        /**
         * Rerun a previously declared task with some name.
         * @param taskName The name of the task to repeat
         * @return The builder object, to allow for chaining.
         * @throws IllegalArgumentException If a task name is passed that doesn't exist.
         */
        public Builder rerunTask(String taskName) throws IllegalArgumentException {
            for (int i = tasks.size()-1; i >= 0; i--) {
                P2PTask task = tasks.get(i);
                if (taskName.equals(task.getName())) {
                    P2PTask newTask = new P2PTask(task);
                    tasks.add(newTask);
                    return this;
                }
            }

            if (useVerboseErrors) {
                throw new IllegalArgumentException("There is no task with name \"" + taskName + "\"!");
            } else {
                return this;
            }
        }

        /**
         * Finish all the currently running actions.
         * @return The builder object, to allow for chaining.
         */
        public Builder finishActions() {
            if (onlyDoPathing) {
                return this;
            }

            P2PTask.Context context = new P2PTask.Context(Robot.drivetrain);
            P2PTask task = new P2PTask(
                    "Finish current actions",
                    context,
                    (P2PTask.Context ctx) -> true,
                    (P2PTask.Context ctx) -> {},
                    P2PTask.Type.FINISH_ACTIONS,
                    0
            );
            tasks.add(task);
            return this;
        }

        /**
         * Creates a follower from the builder.
         * @return A new instance of the follower class.
         */
        public P2PFollower build() {
            return new P2PFollower(new LinkedList<>(tasks), Robot.drivetrain, Robot.telemetry);
        }
    }

    private final Queue<P2PTask> pendingTasks;
    private final ArrayList<P2PTask> runningTasks = new ArrayList<>();
    private final ArrayList<P2PTask> backgroundTasks = new ArrayList<>();
    private final MultipleTelemetry telemetry;
    private final Drivetrain drivetrain;

    private P2PFollower(Queue<P2PTask> tasks, Drivetrain drivetrain, MultipleTelemetry telemetry) {
        this.pendingTasks = tasks;
        this.telemetry = telemetry;
        this.drivetrain = drivetrain;
    }

    public void printTasks(MultipleTelemetry telemetry) {
        for (P2PTask task : pendingTasks) {
            telemetry.addData("task", task.getName());
        }
        telemetry.update();
    }

    public void follow(BooleanSupplier opModeIsActive, @NonNull Supplier<Pose2d> currentPos, Runnable skibidiReading, Runnable update) {
        this.follow(opModeIsActive, currentPos, skibidiReading, update, () -> false, () -> {});
    }

    public void follow(BooleanSupplier opModeIsActive, @NonNull Supplier<Pose2d> currentPos, Runnable skibidiReading, Runnable update, BooleanSupplier shouldKillProgram, Runnable killAction) {
        PidToPoint p2p = new PidToPoint(currentPos.get(), new Pose2d(0.5, 0.5, Math.toRadians(2)), 100);
        ElapsedTime loopTimer = new ElapsedTime();

        while (!(pendingTasks.isEmpty() && runningTasks.isEmpty() && backgroundTasks.isEmpty()) && opModeIsActive.getAsBoolean()) {
            // clear bulk cache (assume manual is being used)
            for (LynxModule hub : RobotHardware.allHubs) {
                hub.clearBulkCache();
            }
            telemetry.clearAll();
            skibidiReading.run();

            boolean needToCallP2p = true;
            Pose2d robotPosition = currentPos.get();
            if (robotPosition.isNaN() || shouldKillProgram.getAsBoolean()) {
                killAction.run();
                telemetry.addLine("Wtf! Localization returned NaN!");
                telemetry.addData("Current Cooked Position", "("+robotPosition.x+", "+robotPosition.y+", "+robotPosition.heading+")");
                telemetry.update();
                continue;
            }

            for (int i = runningTasks.size()-1; i >= 0; i--) {
                P2PTask task = runningTasks.get(i);

                // TODO: this is EXTREMELY inefficient, pls change
                String previousTask = task.getPreviousTask();
                if (previousTask != null) {
                    boolean shouldNotRunYet = false;
                    for (P2PTask task2 : runningTasks) {
                        if (task2.getName().equals(previousTask)) {
                            shouldNotRunYet = true;
                            break;
                        }
                    }
                    for (P2PTask task2 : backgroundTasks) {
                        if (task2.getName().equals(previousTask)) {
                            shouldNotRunYet = true;
                            break;
                        }
                    }

                    if (shouldNotRunYet) {
                        task.getContext().startTimer();
                        continue;
                    }
                }


                if (task.getTaskType() == P2PTask.Type.SLEEP && runningTasks.size() > 1) {
                    task.getContext().startTimer();
                    continue;
                }

                P2PTask.Context ctx = task.getContext();
                ctx.setFollower(p2p);
                ctx.setCurrentPos(robotPosition);

                boolean stalled = task.taskHasStalled();
                if (task.getTaskType() == Task.Type.DRIVING) {
                    needToCallP2p = false;
                }
                if (task.execute(ctx) || stalled) { // run the task
                    // if the task finished or stalled, remove it from the list
                    runningTasks.remove(i);
                }

                if (stalled) {
                    task.kill(ctx);
                }

                telemetry.addData("Task "+i, task.getName());
            }

            for (int i = backgroundTasks.size()-1; i >= 0; i--) {
                P2PTask task = backgroundTasks.get(i);

                P2PTask.Context ctx = task.getContext();
                ctx.setFollower(p2p);
                ctx.setCurrentPos(currentPos.get());

                boolean stalled = task.taskHasStalled();
                if (task.execute(ctx) || stalled) { // run the task
                    // if the task finished or stalled, remove it from the list
                    backgroundTasks.remove(i);
                }

                if (stalled) {
                    task.kill(ctx);
                }

                telemetry.addData("Background Task "+i, task.getName());
            }

            if (needToCallP2p) {
                p2p.calculate(robotPosition);
            }
            drivetrain.move(p2p.getPowers());
            p2p.resetMovingPowers();

            boolean needToAddTasks = runningTasks.isEmpty();

            P2PTask newTask;
            while (needToAddTasks && !pendingTasks.isEmpty()) {
                newTask = pendingTasks.remove();
                newTask.getContext().startTimer();

                if (newTask.getTaskType() == P2PTask.Type.FINISH_ACTIONS) {
                    break;
                }

                if (newTask.getTaskType() == Task.Type.BG_ACTION) {
                    backgroundTasks.add(newTask);
                } else {
                    runningTasks.add(newTask);
                }

                if (newTask.getTaskType() == P2PTask.Type.SLEEP) {
                    break;
                } else if (newTask.getTaskType() == P2PTask.Type.DRIVING) {
                    PointInfo point = newTask.getPointInfo();
                    p2p.setGoal(
                            point.getGoalPoint(),
                            point.getTolerances(),
                            point.getSpeed(),
                            point.getReachedTime()
                    );
                    break;
                }
            }

            update.run();

            Pose2d goal = p2p.getGoal();
            telemetry.addData("Loop time (ms)", loopTimer.milliseconds());
            telemetry.addData("Loop time (hz)", 1000/loopTimer.milliseconds());
            telemetry.addData("Current Goal Point", "("+goal.x+", "+goal.y+", "+goal.heading+")");
            telemetry.addData("Current Position", "("+robotPosition.x+", "+robotPosition.y+", "+robotPosition.heading+")");
            telemetry.update();
            loopTimer.reset();
        }
    }
}

