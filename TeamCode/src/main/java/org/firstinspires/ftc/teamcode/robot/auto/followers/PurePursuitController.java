package org.firstinspires.ftc.teamcode.robot.auto.followers;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Coordinate;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.geometry.Circle;
import org.firstinspires.ftc.teamcode.math.geometry.Ellipse;
import org.firstinspires.ftc.teamcode.math.geometry.LineSegment;
import org.firstinspires.ftc.teamcode.robot.auto.pathgen.QSplines;
import org.firstinspires.ftc.teamcode.robot.drive.Drivetrain;

import java.util.ArrayList;
import java.util.function.Supplier;

/**
 * A class which is used to follow paths using the Pure Pursuit algorithm.
 */
public class PurePursuitController {
    private final double goalThreshold;
    private Coordinate[] pathArr;
    private ArrayList<Obstacle> obstacles = new ArrayList<>();
    private QSplines path;
    private Drivetrain drivetrain;

    public PurePursuitController(Drivetrain drivetrain, double goalThreshold) {
        this.drivetrain = drivetrain;
        this.goalThreshold = goalThreshold;
    }

    /**
     * Adds an obstacle
     * @param obstacle The obstacle to add
     */
    public void addObstacle(Obstacle obstacle) {
        obstacles.add(obstacle);
    }

    /**
     * Sets the path to follow
     * @param path The path to follow.
     * @param points How many points to generate along the path.
     */
    public void setPath(@NonNull QSplines path, int points) {
        this.pathArr = path.generateArr(points);
    }

    /**
     * Follow a path that was set by [.setPath] earlier and drive using the
     * drivetrain set by [.setDrivetrain].
     *
     * @param getCurrentPos A method which retrieves the current position using odometry.
     */
    public void follow(@NonNull Supplier<Pose2d> getCurrentPos) {
        Intersection lastGoalPoint = new Intersection(pathArr[0].toPose(), 0);
        int lastPathIndex = 0;
        Pose2d currentPos = getCurrentPos.get();

        while (Math.abs(Coordinate.distToPoint(Coordinate.fromPose(currentPos), pathArr[pathArr.length - 1])) > goalThreshold) {
            Intersection goalPoint = findAdaptiveEllipseIntersection(lastPathIndex, 0.5, 5.0, currentPos);
            if (goalPoint == null) { // If no new point is found, default to the last goal point
                goalPoint = lastGoalPoint;
            }

            lastGoalPoint = goalPoint;
            lastPathIndex = goalPoint.pathSegmentIndex;

            driveToPoint(currentPos, goalPoint.point);
            currentPos = getCurrentPos.get();
        }
        driveToPoint(currentPos, pathArr[pathArr.length - 1].toPose());
    }

    /**
     * This closed-loop follower uses the standard pure pursuit controller with a fixed circular
     * lookahead. <br>
     * Do not use this, it is merely for demonstration.
     *
     * @param getCurrentPos A method which retrieves the current position of the robot.
     * @param lookAheadDistance The lookahead distance (radius of the circle).
     */
    public void normalFollow(@NonNull Supplier<Pose2d> getCurrentPos, double lookAheadDistance) {
        Coordinate lastGoalPoint = pathArr[0];
        int lastPathIndex = 0;
        Pose2d currentPos = getCurrentPos.get();

        while (Math.abs(Coordinate.distToPoint(Coordinate.fromPose(currentPos), pathArr[pathArr.length - 1])) > goalThreshold) {
            Coordinate goalPoint = null;

            for (int i = lastPathIndex; i < pathArr.length - 1; i++) {
                LineSegment pathSegment = new LineSegment(pathArr[i], pathArr[i + 1]);
                goalPoint = findCircleGoalPoint(pathSegment, Coordinate.fromPose(currentPos), lookAheadDistance);
                if (goalPoint != null) {
                    lastPathIndex = i;
                    break;
                }
            }

            if (goalPoint == null) { // If no new goal point is found, default to the last goal point
                goalPoint = lastGoalPoint;
            }
            lastGoalPoint = goalPoint;

            driveToPoint(currentPos, goalPoint.toPose());
            currentPos = getCurrentPos.get();
        }

        driveToPoint(currentPos, pathArr[pathArr.length - 1].toPose());
    }

    /**
     * Find the goal point using non-adaptive circles.
     *
     * @param path A line segment representing the path.
     * @param robotPosition The robot position (center of the circle).
     * @param lookaheadDistance The lookahead distance (radius of the circle).
     * @return Null if no intersection is found, else the coordinate of intersection.
     */
    private Coordinate findCircleGoalPoint(LineSegment path, Coordinate robotPosition, double lookaheadDistance) {
        Circle circle = new Circle(robotPosition, lookaheadDistance);
        return circle.findNearestIntersection(path);
    }

    /**
     * Drives in the direction of a point
     *
     * @param currentPos The current position of the robot
     * @param goalPoint The point to drive towards
     */
    private void driveToPoint(Pose2d currentPos, @NonNull Pose2d goalPoint) {
        if (goalPoint.heading < 0) {
            goalPoint.heading += 2 * Math.PI;
        }
        double angularErr = Angle.angleWrap(goalPoint.heading - currentPos.heading);
        if (drivetrain != null) {
            drivetrain.move(
                    new Coordinate(goalPoint.x - currentPos.x, goalPoint.y - currentPos.y),
                    angularErr
            );
        }
    }

    /**
     * Check if the robot when driving to a point will crash into any obstacles
     *
     * @param currentPos The current position of the robot
     * @param goalPoint The point the robot is driving towards
     * @return A boolean indicating whether or not the robot would crash into any obstacles
     */
    private boolean crashDetect(Pose2d currentPos, Coordinate goalPoint) {
        // first check if out of bounds
        for (Obstacle obstacle : this.obstacles) {
            // cycling for crash with all of the obstacles
            if (obstacle.crash_detect(currentPos.x, currentPos.y, goalPoint.x, goalPoint.y, currentPos.heading)) {
                return true;
            }
        }
        return false;

        // add obstacle object and use the line intersection test on all four sides of the rectangle objects bounds
    }

    /**
     * Using adaptive ellipses, find the point to drive towards.
     *
     * @param minIndex The minimum index of a segment on the path to start looking from
     * @param minLookAhead The minimum lookahead
     * @param maxLookAhead The maximum lookahead
     * @param currentPos The current position of the robot
     * @return The point to drive towards and the index of the line segment on the path that it's
     *         from, returns null if no point was found.
     */
    @Nullable
    private Intersection findAdaptiveEllipseIntersection(int minIndex, double minLookAhead, double maxLookAhead, Pose2d currentPos) {
        // binary search for the lookahead distance and if the lookahead doesnt work(use crash_detect) please decrease the lookahead
        double stepSize = 100.0;
        double midLookAhead;
        Coordinate goalPoint = null;
        int foundPathIndex = -1;

        while (minLookAhead <= maxLookAhead) {
            midLookAhead = (minLookAhead + maxLookAhead) / 2;
            Ellipse robotEllipse = new Ellipse(
                    Coordinate.fromPose(currentPos),
                    midLookAhead,
                    midLookAhead * 0.5, // TODO: determine minor axis
                    currentPos.heading
            );

            Coordinate intersection = null;
            for (int i = minIndex; i < pathArr.length - 1; i++) {
                LineSegment pathSegment = new LineSegment(pathArr[i], pathArr[i + 1]);
                intersection = robotEllipse.findNearestIntersection(pathSegment);

                if (intersection != null) {
                    foundPathIndex = i;
                    break;
                }
            }

            // if the ellipse is too tiny to intersect (intersection==null) or there wasn't a crash
            // increase the lookahead
            if (intersection == null || !crashDetect(currentPos, intersection)) {
                minLookAhead = midLookAhead + stepSize;

                if (intersection != null) { // if there was an intersection and it didn't crash
                    goalPoint = intersection;
                }
            } else { // if there was an intersection but it crashes
                maxLookAhead = midLookAhead - stepSize;
            }
        }

        if (goalPoint == null) {
            return null;
        }
        return new Intersection(goalPoint.toPose(), foundPathIndex);
    }

    /**
     * A single intersection of an ellipse and the path
     */
    private static class Intersection {
        Pose2d point;
        int pathSegmentIndex;

        Intersection(Pose2d point, int pathSegmentIndex) {
            this.point = point;
            this.pathSegmentIndex = pathSegmentIndex;
        }
    }
}
