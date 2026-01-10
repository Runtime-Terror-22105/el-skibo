package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
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

import java.lang.reflect.Field;

@Config
public class FtcDashDrawing {
    public static double FIELD_ROTATION_DEGREES = 90;
    public static double ROBOT_RADIUS = 9.0;
    private static TelemetryPacket packet;

    static {
        try {
            Canvas newDefaultField = new Canvas();
            newDefaultField.setAlpha(0.4);
            newDefaultField.drawImage("/dash/decode.webp", 0, 0, 144, 144, Math.toRadians(270), 72, 72, false);
            newDefaultField.setAlpha(1.0);
            newDefaultField.drawGrid(0, 0, 144, 144, 7, 7);

            Field defaultField = TelemetryPacket.class.getDeclaredField("DEFAULT_FIELD");
            defaultField.setAccessible(true);
            defaultField.set(null, newDefaultField);
        } catch (NoSuchFieldException | IllegalAccessException e) {
            e.printStackTrace();
        }
    }

    public FtcDashDrawing() {
    }

    private static double[][] convertPointsToFTC(double[][] pedroPoints) {
        double[][] result = new double[2][pedroPoints[0].length];
        for (int i = 0; i < pedroPoints[0].length; i++) {
            Pose pedroPose = new Pose(pedroPoints[0][i], pedroPoints[1][i], 0);
            Pose ftcPose = pedroPose.getAsCoordinateSystem(FTCCoordinates.INSTANCE);
            result[0][i] = ftcPose.getX();
            result[1][i] = ftcPose.getY();
        }
        return result;
    }

    private static void maybeInitFieldPacket() {
        if (packet == null) {
            packet = new TelemetryPacket();
        }
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

    public static void drawDot(Pose pose, String color) {
        maybeInitFieldPacket();

        pose = pose.getAsCoordinateSystem(FTCCoordinates.INSTANCE);
        packet.fieldOverlay().setStroke(color);
        packet.fieldOverlay().setFill(color);
        packet.fieldOverlay().fillCircle(pose.getX(), pose.getY(), 1.0);
    }

    /**
     * Draws a pose as a lttle robot on FTC dashboard wth a certan color.
     * @param pose The pose to draw
     * @param color The color, a string hex code in the format "#AAAAAA", a hash followed by 6 hex digits
     */
    public static void drawRobot(Pose pose, String color) {
        maybeInitFieldPacket();

        packet.fieldOverlay().setStroke(color);
        drawRobotOnCanvas(packet.fieldOverlay(), pose.copy());
    }

    public static void drawPath(Path path, String color) {
        maybeInitFieldPacket();

        packet.fieldOverlay().setStroke(color);
        drawPath(packet.fieldOverlay(), convertPointsToFTC(path.getPanelsDrawingPoints()));
    }

    public static void drawPath(PathChain pathChain, String color) {
        for(int i = 0; i < pathChain.size(); ++i) {
            drawPath(pathChain.getPath(i), color);
        }

    }

    public static void drawPoseHistory(PoseHistory poseTracker, String color) {
        maybeInitFieldPacket();

        // Convert pose x and y to FTC coords
        double[][] ftcPoints = convertPointsToFTC(new double[][]{poseTracker.getXPositionsArray(), poseTracker.getYPositionsArray()});

        packet.fieldOverlay().setStroke(color);
        packet.fieldOverlay().strokePolyline(ftcPoints[0], ftcPoints[1]);
    }

    public static boolean sendPacket() {
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
