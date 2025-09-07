package org.firstinspires.ftc.teamcode.math.geometry;

import static org.firstinspires.ftc.teamcode.math.Coordinate.ORIGIN;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.math.Algebra;
import org.firstinspires.ftc.teamcode.math.Coordinate;
import org.jetbrains.annotations.Contract;

public class LineSegment {
    public Coordinate bound1;
    public Coordinate bound2;

    public LineSegment(Coordinate point1, Coordinate point2) {
        this.bound1 = point1;
        this.bound2 = point2;
    }

    public double getSlope() {
        return (bound2.y - bound1.y)/(bound2.x - bound1.x);
    }

    public double getYIntercept() {
        return bound1.y - getSlope()*bound1.x;
    }

    public double getYIntercept(double slope) {
        // a version where you pass in the slope in case you already calculated it
        return bound1.y - slope*bound1.x;
    }

    public double solveForY(double x) {
        double m = getSlope();
        return m*x + getYIntercept(m);
    }

    public static double solveForY(double x, double slope, double yIntercept) {
        // a version where you pass in the slope and y-int if already calculated
        return slope*x + yIntercept;
    }

    /**
     * Translate a line segment by some x and y.
     * @param x The amount to translate the line segment on the x-axis.
     * @param y The amount to translate the line segment on the y-axis.
     */
    public void translate(double x, double y) {
        this.bound1.translate(x, y);
        this.bound2.translate(x, y);
    }

    /**
     * Translate a line segment by some x and y.
     * @param coord The coordinate to translate by.
     */
    public void translate(@NonNull Coordinate coord) {
        this.bound1.translate(coord.x, coord.y);
        this.bound2.translate(coord.x, coord.y);
    }

    /**
     * Translate a line segment by some x and y.
     * @param x The amount to translate the line segment on the x-axis.
     * @param y The amount to translate the line segment on the y-axis.
     */
    @NonNull
    @Contract("_, _, _ -> new")
    public static LineSegment translate(@NonNull LineSegment lineSegment, double x, double y) {
        return new LineSegment(
            Coordinate.translate(lineSegment.bound1, x, y),
            Coordinate.translate(lineSegment.bound2, x, y)
        );
    }

    /**
     * Translate a line segment by some x and y.
     * @param coord The coordinate to translate by.
     */
    @NonNull
    @Contract("_, _ -> new")
    public static LineSegment translate(@NonNull LineSegment lineSegment, @NonNull Coordinate coord) {
        return new LineSegment(
                Coordinate.plus(lineSegment.bound1, coord),
                Coordinate.plus(lineSegment.bound2, coord)
        );
    }

    /**
     * Rotate the line segment.
     * @param angle An angle, in radians.
     * @return The rotated line segment.
     */
    @NonNull
    @Contract("_, _ -> new")
    public static LineSegment rotate(@NonNull LineSegment segment, double angle) {
        return new LineSegment(
            Coordinate.rotate(segment.bound1, ORIGIN, angle),
            Coordinate.rotate(segment.bound2, ORIGIN, angle)
        );
    }

    /**
     * Rotate the line segment.
     * @param angle An angle, in radians.
     */
    public void rotate(double angle) {
        this.rotate(ORIGIN, angle);
    }

    /**
     * Rotate the line segment around a point.
     * @param center The center of rotation.
     * @param angle An angle, in radians.
     */
    public void rotate(Coordinate center, double angle) {
        this.bound1.rotate(center, angle);
        this.bound2.rotate(center, angle);
    }

    public boolean isVertical() {
        return bound2.x == bound1.x;
    }

    public boolean isWithinBounds(@NonNull Coordinate point) {
        double minX = Math.min(bound1.x, bound2.x);
        double maxX = Math.max(bound1.x, bound2.x);
        double minY = Math.min(bound1.y, bound2.y);
        double maxY = Math.max(bound1.y, bound2.y);

        return point.x >= minX && point.x <= maxX && point.y >= minY && point.y <= maxY;
    }

    /**
     * Returns whether a point is on the line segment.
     * @param point The point.
     * @return Whether or not the point sits on the shape.
     */
    public boolean isValidSolution(Coordinate point) {
        boolean onLine;
        if (isVertical()) {
            onLine = point.x == bound1.x;
        }
        else {
            double slope = getSlope();
            onLine = Algebra.round(point.y, 5) == Algebra.round(slope * point.x + getYIntercept(slope), 5);
        }

        return isWithinBounds(point) && onLine;
    }
}
