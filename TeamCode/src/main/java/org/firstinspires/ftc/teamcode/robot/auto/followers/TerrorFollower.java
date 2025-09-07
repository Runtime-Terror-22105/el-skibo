package org.firstinspires.ftc.teamcode.robot.auto.followers;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.math.Pose2d;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Predicate;
import java.util.function.Supplier;

public interface TerrorFollower {
    interface Builder {
        /**
         * Execute some action once.
         * @param taskName The name of the task in logs
         * @param action The function to run.
         * @param timeLimit How long the action can take to run (ms), before it is considered to have stalled.
         * @return The builder object, to allow for chaining.
         */
        Builder executeActionOnce(String taskName, Consumer<Task.Context> action, double timeLimit);

        /**
         * Simply keeps the drivetrain active and runs the pre/after loop funcs certain duration of time.
         * @param milliseconds How long to wait for.
         * @return The builder object, to allow for chaining.
         */
        Builder sleep(int milliseconds);

        /**
         * Execute some action repeatedly (async) until some condition is true.
         * @param taskName The name of the task to run.
         * @param condition The condition.
         * @param action The function to run.
         * @param timeLimit How long the action can take to run (ms), before it is considered to have stalled.
         * @return The builder object, to allow for chaining.
         */
        Builder executeUntilTrue(
                String taskName,
                Predicate<Task.Context> condition,
                Consumer<Task.Context> action,
                double timeLimit
        );

        /**
         * Run some action for some amount of time.
         * @param taskName The name of the task to run.
         * @param action The function to run.
         * @param timeLimit The amount of time, in milliseconds
         * @return The builder object, to allow for chaining.
         */
        Builder executeForTime(
                String taskName,
                Consumer<Task.Context> action,
                double timeLimit
        );

        /**
         * Rerun a previously declared task with some name.
         * @param taskName The name of the task to repeat
         * @return The builder object, to allow for chaining.
         * @throws IllegalArgumentException If a task name is passed that doesn't exist.
         */
        Builder rerunTask(String taskName) throws IllegalArgumentException;

        /**
         * Finish all the currently running actions.
         * @return The builder object, to allow for chaining.
         */
        Builder finishActions();

        /**
         * Creates a follower from the builder.
         * @return A new instance of the follower class.
         */
        TerrorFollower build();
    }

    void follow(BooleanSupplier opModeIsActive, @NonNull Supplier<Pose2d> currentPos, Runnable skibidiReading, Runnable update);

}
