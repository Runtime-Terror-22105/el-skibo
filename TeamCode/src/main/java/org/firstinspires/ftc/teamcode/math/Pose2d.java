package org.firstinspires.ftc.teamcode.math;

import androidx.annotation.NonNull;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.jetbrains.annotations.Contract;

public class Pose2d {
    public double x;
    public double y;
    public double heading;

    public Pose2d() {
        this(0, 0, 0);
    }

    public Pose2d(Pose pose) {
        this(pose.getX(), pose.getY(), pose.getHeading());
    }

    public Pose2d(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose2d(@NonNull SparkFunOTOS.Pose2D pose) {
        this(pose.x, pose.y, pose.h);
    }

    public Pose2d(@NonNull org.opencv.core.Point point, double heading) {
        this(point.x, point.y, heading);
    }

    public Pose2d(@NonNull Pose2D pose) {
        this(pose.getX(DistanceUnit.MM), pose.getY(DistanceUnit.MM), pose.getHeading(AngleUnit.RADIANS));
    }

    @NonNull
    @Contract(value = "_, _ -> new", pure = true)
    public static Pose2d add(@NonNull Pose2d pose1, @NonNull Pose2d pose2) {
        return new Pose2d(
                pose1.x+ pose2.x,
                pose1.y+pose2.y,
                Angle.angleWrap(pose1.heading+pose2.heading)
        );
    }

    public SparkFunOTOS.Pose2D toOtosPose() {
        return new SparkFunOTOS.Pose2D(this.x, this.y, this.heading);
    }

    public Pose toPedro() {
        return new Pose(this.x, this.y, this.heading);
    }

    public Pose2d mult(double scalar) {
        this.x *= scalar;
        this.y *= scalar;
        this.heading *= scalar;
        return this;
    }

    public boolean isNaN() {
        return Double.isNaN(this.x) || Double.isNaN(this.y) || Double.isNaN(this.heading);
    }

    public Pose2d plus(@NonNull Pose2d other) {
        return new Pose2d(
                this.x+other.x,
                this.y+other.y,
                this.heading+other.heading
        );
    }

    public void minus(@NonNull Coordinate other) {
            this.x -= other.x;
            this.y -= other.y;
    }

    @NonNull
    @Override
    public String toString() {
        return "(" + x + ", " + y + ", " + heading + ")";
    }

    public Pose2d copy() {
        return new Pose2d(
                this.x,
                this.y,
                this.heading
        );
    }
}
