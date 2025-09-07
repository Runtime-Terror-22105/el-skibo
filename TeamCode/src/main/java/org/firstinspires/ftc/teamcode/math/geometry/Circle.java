package org.firstinspires.ftc.teamcode.math.geometry;

import static org.firstinspires.ftc.teamcode.math.Algebra.sign;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.firstinspires.ftc.teamcode.math.Algebra;
import org.firstinspires.ftc.teamcode.math.Coordinate;

public class Circle implements Shape {
    private final Coordinate center;
    private final double radius;

    public Circle(Coordinate center, double radius) {
        this.center = center;
        this.radius = radius;
    }

    /**
     * Returns whether a point is on a shape.
     * @param point The point.
     * @return Whether or not the point sits on the shape.
     */
    @Override
    public boolean isValidSolution(@NonNull Coordinate point) {
        double eqnLeftSide = Math.pow((point.x-center.x), 2) + Math.pow((point.y-center.y), 2);
        return Algebra.round(eqnLeftSide, 5) == radius*radius;
    }

    /**
     * Get an intersection between a line segment and circle. If there are two, the point closer
     * to the second point will be chosen. If there are none, null will be returned.
     * @param lineSegment The line segment
     * @return A point, can be null.
     */
    @Nullable
    public Coordinate findNearestIntersection(@NonNull LineSegment lineSegment) {
        // Math comes from https://mathworld.wolfram.com/Circle-LineIntersection.html, except we
        // shift the center of the circle to the origin since our circle won't be centered at the
        // origin. We also add an extra check that the circle intersects with the line segmnet, not
        // the entire line.
        lineSegment.translate(Coordinate.mirror(center)); // shift to origin

        double dx = lineSegment.bound2.x - lineSegment.bound1.x;
        double dy = lineSegment.bound2.y - lineSegment.bound1.y;
        double dr = Math.sqrt(dx * dx + dy * dy);
        double det = lineSegment.bound1.x * lineSegment.bound2.y - lineSegment.bound2.x * lineSegment.bound1.y;
        double discriminant = Math.pow(radius, 2) * Math.pow(dr, 2) - Math.pow(det, 2);

        if (discriminant < 0) {
            return null;
        }

        double temp = sign(dy) * dx * Math.sqrt(discriminant);
        double x1 = (det * dy + temp) / Math.pow(dr, 2);
        double x2 = (det * dy - temp) / Math.pow(dr, 2);

        temp = Math.abs(dy) * Math.sqrt(discriminant);
        double y1 = (-det * dx + temp) / Math.pow(dr, 2);
        double y2 = (-det * dx - temp) / Math.pow(dr, 2);

        Coordinate sol1 = new Coordinate(x1, y1).plus(this.center);
        Coordinate sol2 = new Coordinate(x2, y2).plus(this.center);

        // Set the solution to null if it is outside of the line segment
        if (!(lineSegment.isValidSolution(sol1) && this.isValidSolution(sol1))) {
            sol1 = null;
        }
        else if (!(lineSegment.isValidSolution(sol2) && this.isValidSolution(sol2))) {
            sol2 = null;
        }

        // Return the valid solution, or if there are two, the one closer to point 2
        return lineSegment.bound2.closestPoint(sol1, sol2);
    }
}
