package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;

public class FtcDashDrawing {
    public static final double ROBOT_RADIUS = 9.0;
    private static TelemetryPacket packet;

    public FtcDashDrawing() {
    }

    public static void drawDebug(Follower follower) {
        if (follower.getCurrentPath() != null) {
            drawPath(follower.getCurrentPath(), "#3F51B5");
            Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
            drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(), follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), "#3F51B5");
        }

        drawPoseHistory(follower.getPoseHistory(), "#4CAF50");
        drawRobot(follower.getPose(), "#4CAF50");
        sendPacket();
    }

    public static void drawRobot(Pose pose, String color) {
        if (packet == null) {
            packet = new TelemetryPacket();
        }

        packet.fieldOverlay().setStroke(color);
        drawRobotOnCanvas(packet.fieldOverlay(), pose.copy());
    }

    public static void drawPath(Path path, String color) {
        if (packet == null) {
            packet = new TelemetryPacket();
        }

        packet.fieldOverlay().setStroke(color);
        drawPath(packet.fieldOverlay(), path.getPanelsDrawingPoints());
    }

    public static void drawPath(PathChain pathChain, String color) {
        for(int i = 0; i < pathChain.size(); ++i) {
            drawPath(pathChain.getPath(i), color);
        }

    }

    public static void drawPoseHistory(PoseHistory poseTracker, String color) {
        if (packet == null) {
            packet = new TelemetryPacket();
        }

        packet.fieldOverlay().setStroke(color);
        packet.fieldOverlay().strokePolyline(poseTracker.getXPositionsArray(), poseTracker.getYPositionsArray());
    }

    private static boolean sendPacket() {
        if (packet != null) {
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            packet = null;
            return true;
        } else {
            return false;
        }
    }

    private static void drawRobotOnCanvas(Canvas c, Pose t) {
        t = t.getAsCoordinateSystem(FTCCoordinates.INSTANCE);
        c.strokeCircle(t.getX(), t.getY(), 9.0);
        Vector v = t.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * 9.0);
        double x1 = t.getX() + v.getXComponent() / 2.0;
        double y1 = t.getY() + v.getYComponent() / 2.0;
        double x2 = t.getX() + v.getXComponent();
        double y2 = t.getY() + v.getYComponent();
        c.strokeLine(x1, y1, x2, y2);
    }

    public static void drawPath(Canvas c, double[][] points) {
        c.strokePolyline(points[0], points[1]);
    }
}
