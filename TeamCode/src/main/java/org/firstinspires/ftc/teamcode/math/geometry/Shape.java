package org.firstinspires.ftc.teamcode.math.geometry;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.math.Coordinate;

public interface Shape {
    /**
     * Returns whether a point is on a shape.
     * @param point The point.
     * @return Whether or not the point sits on the shape.
     */
    boolean isValidSolution(Coordinate point);

    Coordinate findNearestIntersection(@NonNull LineSegment lineSegment);
}
