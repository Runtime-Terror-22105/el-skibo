package org.firstinspires.ftc.teamcode.math.geometry;

import static org.firstinspires.ftc.teamcode.math.Coordinate.ORIGIN;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.firstinspires.ftc.teamcode.math.Algebra;
import org.firstinspires.ftc.teamcode.math.Coordinate;
import org.jetbrains.annotations.Contract;

public class Ellipse implements Shape {
    private final Coordinate center;
    public double major;// major axis
    public double minor; // minor axis
    public double rotation;

    public Ellipse(Coordinate center, double major, double minor, double rotation) {
        this.major = major;
        this.minor = minor;
        this.center = center;
        this.rotation = rotation;
    }

    public Ellipse(double major, double minor) {
        this.major = major;
        this.minor = minor;
        this.center = ORIGIN;
        this.rotation = 0;
    }

    /**
     * Get an intersection between a line segment and a ellipse. If there are two, the point closer
     * to the second point will be chosen. If there are none, null will be returned.
     * @param lineSegment The line segment.
     * @return A point, can be null.
     */
    @Nullable
    public Coordinate findNearestIntersection(@NonNull LineSegment lineSegment) {
        // Rather than rotating the ellipse, we can just shift the ellipse to the origin and rotate
        // the entire axis. This means that we end up not rotating the ellipse and we rotate the
        // line clockwise (negative rotation).

        lineSegment.translate(Coordinate.mirror(center)); // shift to origin
        lineSegment.rotate(-rotation); // rotate backwards
        lineSegment.translate(center); // shift back

        Coordinate intersection = unrotatedEllipseLineIntersection(lineSegment);
        intersection.rotate(rotation);
        return intersection;
    }

    /**
     * Get an intersection between a line segment and a ellipse. If there are two, the point closer
     * to the second point will be chosen. If there are none, null will be returned.
     * @param lineSegment The line segment.
     * @return A point, can be null.
     */
    @Nullable
    @Contract("_-> new")
    public Coordinate unrotatedEllipseLineIntersection(@NonNull LineSegment lineSegment) {
        // Math comes from https://youtu.be/4W4BiRPTDKs?si=I7sFWELk39a1NFC9, except we
        // shift the center of the ellipse to the origin since our ellipse won't be centered at the
        // origin. We also add an extra check that the circle intersects with the line segment, not
        // the entire line.
        lineSegment.translate(Coordinate.mirror(center)); // shift to origin

        Coordinate sol1, sol2;

        // if the case that the bottom is 0 check if the x-axis passes anywhere from center+-minor/2

        double mjSqr = Math.pow(major, 2); // major axis squared
        double mnSqr = Math.pow(minor, 2); //minor axis squared

        if (lineSegment.isVertical()) { // infinite slope case
            double xSqr = Math.pow(lineSegment.bound1.x, 2);
            sol1 = new Coordinate(
                    lineSegment.bound1.x,
                    minor/major * Math.sqrt(mjSqr - xSqr)
            );
            sol2 = new Coordinate(
                    lineSegment.bound1.x,
                    -minor/major * Math.sqrt(mjSqr - xSqr)
            );
        } else { // normal case
            double slope = lineSegment.getSlope();
            double yIntercept = lineSegment.getYIntercept(slope);

            double slSqr = Math.pow(slope, 2); //slope squared
            double intSqr = Math.pow(yIntercept, 2); // intercept squared
            Algebra.QuadraticFormulaResult quadraticRoots = Algebra.solveQuadraticRoots(
                    mnSqr + mjSqr * slSqr,
                    2 * mjSqr * slope * yIntercept,
                    mjSqr * (intSqr - mnSqr)
            );

            if (quadraticRoots.answerCount == 0) { // no intersection
                return null;
            }

            double x1 = quadraticRoots.root1;
            double x2 = quadraticRoots.root2;

            double y1 = LineSegment.solveForY(x1, slope, yIntercept);
            double y2 = LineSegment.solveForY(x2, slope, yIntercept);

            sol1 = new Coordinate(x1, y1);
            sol2 = new Coordinate(x2, y2);
        }

        sol1.plus(center);
        sol2.plus(center);

        double oldRotation = rotation;
        rotation = 0; // set rotation to 0 temporarily when checking if valid sln
        // Set the solution to null if it is outside of the line segment or not on the ellipse
        if (!(lineSegment.isValidSolution(sol1) && this.isValidSolution(sol1))) {
            sol1 = null;
        }
        else if (!(lineSegment.isValidSolution(sol2) && this.isValidSolution(sol2))) {
            sol2 = null;
        }
        rotation = oldRotation;

        // Return the valid solution, or if there are two, the one closer to point 2
        return lineSegment.bound2.closestPoint(sol1, sol2);
    }

    /**
     * Returns whether a point is on the ellipse.
     * @param point The point.
     * @return Whether or not the point sits on the shape.
     */
    @Override
    public boolean isValidSolution(@NonNull Coordinate point) {
        // eqn of rotated ellipse is in eqn 6, page 9: http://quickcalcbasic.com/ellipse%20line%20intersection.pdf#page=4.09
        double cosAngle = Math.cos(rotation);
        double sinAngle = Math.sin(rotation);
        double finalXSqr = Math.pow(point.x * cosAngle + point.y * sinAngle, 2);
        double finalYSqr = Math.pow(point.y * cosAngle - point.x * sinAngle, 2);
        double mjSqr = major*major;
        double mnSqr = minor*minor;

        // we round to 5 decimal places due to issues with floating point arithmetic
        return Algebra.round(finalXSqr/mjSqr + finalYSqr/mnSqr, 5) == 1d;
    }
}
