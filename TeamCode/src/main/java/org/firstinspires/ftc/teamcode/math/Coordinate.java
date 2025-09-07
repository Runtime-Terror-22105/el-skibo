package org.firstinspires.ftc.teamcode.math;

import androidx.annotation.NonNull;

import org.jetbrains.annotations.Contract;
import org.opencv.core.Point;

public class Coordinate {
    public static final Coordinate ORIGIN = new Coordinate(0, 0);

    public double x;
    public double y;

    public Coordinate(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Coordinate(@NonNull Point cvPoint) {
        this.x = cvPoint.x;
        this.y = cvPoint.y;
    }

    /**
     * Gets the cartesian distance between two points.
     * @param point1 The first point.
     * @param point2 The second point.
     * @return The distance between the two points.
     */
    public static double distToPoint(@NonNull Coordinate point1, @NonNull Coordinate point2) {
        return Math.sqrt(Math.pow(point2.x - point1.x, 2) + Math.pow(point2.y - point1.y, 2));
    }

    /**
     * Gets the cartesian distance between two points.
     * @param point The other point.
     * @return The distance between the two points.
     */
    public double distToPoint(@NonNull Coordinate point) {
        return Math.sqrt(Math.pow(point.x - this.x, 2) + Math.pow(point.y - this.y, 2));
    }

    /**
     * Checks if a point is within some bounds.
     * @param point The point.
     * @param minX  The minimum x.
     * @param maxX  The maximum x.
     * @param minY  The minimum y.
     * @param maxY  The maximum y.
     * @return Whether or not the point is within the bounds.
     */
    public static boolean isWithinBounds(@NonNull Coordinate point, double minX, double maxX, double minY, double maxY) {
        return point.x >= minX && point.x <= maxX && point.y >= minY && point.y <= maxY;
    }

    /**
     * Checks if a point is within some bounds.
     * @param minX  The minimum x.
     * @param maxX  The maximum x.
     * @param minY  The minimum y.
     * @param maxY  The maximum y.
     * @return Whether or not the point is within the bounds.
     */
    public boolean isWithinBounds(double minX, double maxX, double minY, double maxY) {
        return this.x >= minX && this.x <= maxX && this.y >= minY && this.y <= maxY;
    }

    /**
     * Mirror a coordinate by negating its x and y value.
     * @param coord The coordinate.
     * @return The mirrored coordinate.
     */
    @NonNull
    @Contract(value = "_ -> new", pure = true)
    public static Coordinate mirror(@NonNull Coordinate coord) {
        return new Coordinate(
                -coord.x,
                -coord.y
        );
    }

    /**
     * Mirror a coordinate by negating its x and y value.
     */
    public void mirror() {
        this.x = -this.x;
        this.y = -this.y;
    }

    /**
     * Move a point.
     * @param x The amount to translate the point on the x axis.
     * @param y The amount to translate the point on the y axis.
     */
    public static Coordinate translate(Coordinate coord, double x, double y) {
        return new Coordinate(
                coord.x + x,
                coord.y + y
        );
    }

    /**
     * Move a point.
     * @param x The amount to translate the point on the x axis.
     * @param y The amount to translate the point on the y axis.
     */
    public void translate(double x, double y) {
        this.x += x;
        this.y += y;
    }

    /**
     * Add two vectors.
     * @param v1 The first vector
     * @param v2 The second vector
     * @return A vector, with the elements of both added to each other
     */
    @NonNull
    @Contract(value = "_, _ -> new", pure = true)
    public static Coordinate plus(@NonNull Coordinate v1, @NonNull Coordinate v2) {
        return new Coordinate(v1.x + v2.x, v1.y + v2.y);
    }

    /**
     * Add two vectors.
     * @param other The vector to add
     * @return A new vector, with the elements of both added to each other
     */
    public Coordinate plus(@NonNull Coordinate other) {
        this.x += other.x;
        this.y += other.y;
        return this;
    }

    /**
     * Add two vectors.
     * @param v1 The first vector
     * @param v2 The second vector
     * @return A vector, with the elements of both added to each other
     */
    @NonNull
    @Contract(value = "_, _ -> new", pure = true)
    public static Coordinate minus(@NonNull Coordinate v1, @NonNull Coordinate v2) {
        return new Coordinate(v1.x - v2.x, v1.y - v2.y);
    }

    /**
     * Add two vectors.
     * @param other The vector to add
     * @return A new vector, with the elements of both added to each other
     */
    public Coordinate minus(@NonNull Coordinate other) {
        this.x -= other.x;
        this.y -= other.y;
        return this;
    }

    /**
     * Multiply a vector by a scalar.
     * @param v1 The vector to scale
     * @param scalar The scalar to multiply by
     * @return The scaled vector
     */
    @NonNull
    @Contract(value = "_, _ -> new", pure = true)
    public static Coordinate mult(@NonNull Coordinate v1, double scalar) {
        return new Coordinate(v1.x * scalar, v1.y * scalar);
    }

    /**
     * Multiply a vector by a scalar.
     * @param scalar The scalar to multiply by
     * @param v1 The vector to scale
     * @return The scaled vector
     */
    @NonNull
    @Contract(value = "_, _ -> new", pure = true)
    public static Coordinate mult(double scalar, @NonNull Coordinate v1) {
        return new Coordinate(v1.x * scalar, v1.y * scalar);
    }

    /**
     * Multiply the vector by a scalar.
     * @param scalar The scalar to multiply by
     * @return The scaled vector
     */
    public Coordinate mult(@NonNull double scalar) {
        this.x *= scalar;
        this.y *= scalar;
        return this;
    }


    /**
     * Find the dot product of two vectors.
     * @param v1 The first vector
     * @param v2 The second vector
     * @return The dot product of the vectors
     */
    @NonNull
    @Contract(value = "_, _ -> new", pure = true)
    public static double dot(@NonNull Coordinate v1, @NonNull Coordinate v2) {
        return v1.x * v2.x + v1.y * v2.y;
    }

    /**
     * Rotate the point around the origin by an angle.
     * @param point The point.
     * @param angle Angle, in radians.
     * @return The rotated point.
     */
    @NonNull
    @Contract("_, _, _ -> new")
    public static Coordinate rotate(@NonNull Coordinate point, @NonNull Coordinate center,
                                    double angle) {
        double y  =  (point.x-center.x) * Math.sin(angle) + (point.y-center.y) * Math.cos(angle) + center.y;
        return new Coordinate(
            (point.x-center.x) * Math.cos(angle) - (point.y-center.y) * Math.sin(angle) + center.x,
            y
        );
    }

    /**
     * Rotate the point around the origin by an angle.
     * @param angle Angle, in radians.
     */
    public void rotate(double angle) {
        this.rotate(ORIGIN, angle);
//        double tmp  =  this.x * Math.sin(angle) + this.y * Math.cos(angle);
//        this.x  = this.x * Math.cos(angle) - this.y * Math.sin(angle);
//        this.y  =  tmp;
//        return this;
    }

    /**
     * Rotate the point around some point by an angle.
     * @param center The center of rotation.
     * @param angle Angle, in radians.
     */
    public void rotate(@NonNull Coordinate center, double angle) {
        double tmp  =  (this.x-center.x) * Math.sin(angle) + (this.y-center.y) * Math.cos(angle) + center.y;
        this.x = (this.x-center.x) * Math.cos(angle) - (this.y-center.y) * Math.sin(angle) + center.x;
        this.y = tmp;
    }

    public Coordinate closestPoint(Coordinate point1, Coordinate point2) {
        if (point1 != null && point2 != null) {
            return this.distToPoint(point1) < this.distToPoint(point2) ? point1 : point2;
        } else if (point1 != null) {
            return point1;
        } else {
            return point2;
        }
    }

    /**
     * Add two vectors.
     * @param other The vector to do dot product with
     * @return The dot product of this vector with the other.
     */
    public double dot(@NonNull Coordinate other) {
        return this.x * other.x + this.y * other.y;
    }

    public double magnitude() {
        return Math.sqrt(x*x + y*y);
    }

    @NonNull
    @Contract(value = "_ -> new", pure = true)
    public static Coordinate fromPose(@NonNull Pose2d pose) {
        return new Coordinate(pose.x, pose.y);
    }

    @NonNull
    public Pose2d toPose() {
        return new Pose2d(this.x, this.y, Math.atan2(this.y, this.x));
    }
}
