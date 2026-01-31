package org.firstinspires.ftc.teamcode.robot.auto;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.init.Robot;

public final class PathUtil {
    private PathUtil() {
    }

    public static PathBuilder addPathBuilderLine(Robot robot, Pose2d startPoseIn, Pose2d endPoseIn, boolean mirror, boolean tangentialHeading, boolean reversed) {
        Pose startPose = startPoseIn.toPedro();
        Pose endPose = endPoseIn.toPedro();
        if (mirror) {
            startPose = startPose.mirror();
            endPose = endPose.mirror();
        }

        PathBuilder builder = robot.follower
                .pathBuilder()
                .addPath(
                        new BezierLine(startPose, endPose)
                );

        if (tangentialHeading) {
            builder = builder.setTangentHeadingInterpolation();
        } else {
            builder = builder.setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading());
        }

        if (reversed) {
            builder = builder.setReversed();
        }
        return builder;
    }

    public static PathBuilder addPathBuilderLine(Robot robot, PathChain prevPath, Pose2d endPoseIn, boolean mirror, boolean tangentialHeading, boolean reversed) {
        Pose startPose = prevPath.endPoint();

        // we do this bc we want to use the calculated heading in the path rather than the heading we had set in the path (i.e. for tangential)
        startPose = startPose.setHeading(prevPath.getFinalHeadingGoal());

        // if mirroring, we need to mirror the start pose back to original side first, since prevPath was already mirrored and createLinePath will mirror it again
        return addPathBuilderLine(robot, new Pose2d(startPose).mirror(mirror), endPoseIn, mirror, tangentialHeading, reversed);
    }

    public static PathChain createLinePath(Robot robot, Pose2d startPoseIn, Pose2d endPoseIn, boolean mirror, boolean tangentialHeading, boolean reversed) {
        return addPathBuilderLine(robot, startPoseIn, endPoseIn, mirror, tangentialHeading, reversed).build();
    }

    public static PathChain createLinePath(Robot robot, PathChain prevPath, Pose2d endPoseIn, boolean mirror, boolean tangentialHeading, boolean reversed) {
        return addPathBuilderLine(robot, prevPath, endPoseIn, mirror, tangentialHeading, reversed).build();
    }

    public static PathBuilder addPathBuilderCurve(Robot robot, Pose2d startPoseIn, Pose2d controlPoseIn, Pose2d endPoseIn, boolean mirror, boolean tangentialHeading, boolean reversed) {
        Pose startPose = startPoseIn.toPedro();
        Pose controlPose = controlPoseIn.toPedro();
        Pose endPose = endPoseIn.toPedro();
        if (mirror) {
            startPose = startPose.mirror();
            controlPose = controlPose.mirror();
            endPose = endPose.mirror();
        }

        PathBuilder builder = robot.follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                startPose,
                                controlPose,
                                endPose
                        )
                );

        if (tangentialHeading) {
            builder = builder.setTangentHeadingInterpolation();
        } else {
            builder = builder.setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading());
        }

        if (reversed) {
            builder = builder.setReversed();
        }
        return builder;
    }

    public static PathBuilder addPathBuilderCurve(Robot robot, PathChain prevPath, Pose2d controlPoseIn, Pose2d endPoseIn, boolean mirror, boolean tangentialHeading, boolean reversed) {
        Pose startPose = prevPath.endPoint();

        // we do this bc we want to use the calculated heading in the path rather than the heading we had set in the path (i.e. for tangential)
        startPose = startPose.setHeading(prevPath.getFinalHeadingGoal());

        // if mirroring, we need to mirror the start pose back to original side first, since prevPath was already mirrored and createLinePath will mirror it again
        return addPathBuilderCurve(robot, new Pose2d(startPose).mirror(mirror), controlPoseIn, endPoseIn, mirror, tangentialHeading, reversed);
    }

    public static PathChain createCurvePath(Robot robot, Pose2d startPoseIn, Pose2d controlPoseIn, Pose2d endPoseIn, boolean mirror, boolean tangentialHeading, boolean reversed) {
        return addPathBuilderCurve(robot, startPoseIn, controlPoseIn, endPoseIn, mirror, tangentialHeading, reversed).build();
    }

    public static PathChain createCurvePath(Robot robot, PathChain prevPath, Pose2d controlPoseIn, Pose2d endPoseIn, boolean mirror, boolean tangentialHeading, boolean reversed) {
        return addPathBuilderCurve(robot, prevPath, controlPoseIn, endPoseIn, mirror, tangentialHeading, reversed).build();
    }
}
