package org.firstinspires.ftc.teamcode.robot.auto;

import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.init.Robot;

public class FollowPathAndWaitForWallCommand extends CommandBase {
    private enum State {
        INITIAL_PATH,
        WAITING_FOR_WALL,
        FINAL_PATH,
        COMPLETED
    }

    private final Robot robot;
    private final PathChain pathChain;
    private final boolean holdEnd;
    private final double maxPower;
    private final double wallTimeoutDistance;

    private boolean wallDistanceIsForRemaining;

    private State state = State.INITIAL_PATH;

    /**
     * @param wallTimeoutDistance The wall must be down after this many inches have
     *                            been traveled, otherwise we cancel the path and hold
     *                            until it is down. Call wallDistanceIsForRemaining()
     *                            if you want this to be based on distance remaining instead of distance traveled.
     */
    public FollowPathAndWaitForWallCommand(Robot robot, PathChain pathChain, boolean holdEnd, double maxPower, double wallTimeoutDistance) {
        this.robot = robot;
        this.pathChain = pathChain;
        this.holdEnd = holdEnd;
        this.maxPower = maxPower;
        this.wallTimeoutDistance = wallTimeoutDistance;
        this.wallDistanceIsForRemaining = false;
    }

    /**
     * If this is set, then wallTimeOutDistance will instead be based on the distance remaining rather than distance traveled.
     * This is useful in paths where we're more uncertain of the distance travelled.
     */
    public FollowPathAndWaitForWallCommand wallDistanceIsForRemaining() {
        this.wallDistanceIsForRemaining = true;
        return this;
    }

    @Override
    public void initialize() {
        robot.follower.followPath(pathChain, maxPower, holdEnd);
        state = State.INITIAL_PATH;
    }

    @Override
    public void execute() {
        switch (state) {
            case INITIAL_PATH:
                boolean timeToCheckForWall;
                if (wallDistanceIsForRemaining) {
                    timeToCheckForWall = robot.follower.getDistanceRemaining() <= wallTimeoutDistance;
                } else {
                    timeToCheckForWall = robot.follower.getDistanceTraveledOnPath() >= wallTimeoutDistance;
                }
                if (timeToCheckForWall) {
                    if (robot.spindexer.isWallDown()) {
                        state = State.FINAL_PATH;
                    } else {
                        state = State.WAITING_FOR_WALL;
                        robot.follower.pausePathFollowing();
                    }
                }
                break;
            case WAITING_FOR_WALL:
                if (robot.spindexer.isWallDown()) {
                    state = State.FINAL_PATH;
                    robot.follower.resumePathFollowing();
                }
                break;
            case FINAL_PATH:
                if (!robot.follower.isBusy()) {
                    state = State.COMPLETED;
                }
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return State.COMPLETED.equals(state);
    }
}
