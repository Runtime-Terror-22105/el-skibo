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

import org.firstinspires.ftc.teamcode.robot.init.Robot;

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

    /**
     * Draws a ray from the robot pose in the direction it is facing
     * until it intersects the edge of the FTC field.
     *
     * @param pose  Robot pose (any coordinate system)
     * @param color Stroke color (e.g. "#00FF00")
     */
    public static void drawHeadingRay(Pose pose, String color) {
        maybeInitFieldPacket();

        Pose start = pose.getAsCoordinateSystem(FTCCoordinates.INSTANCE);
        Pose end   = raycastToFieldEdge(pose);

        packet.fieldOverlay().setStroke(color);
        packet.fieldOverlay().strokeLine(
                start.getX(), start.getY(),
                end.getX(),   end.getY()
        );
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

        // Draw a rectangle of Robot.ROBOT_WIDTH and Robot.ROBOT_LENGTH at angle t.getHeading().
        Vector fwd = t.getHeadingAsUnitVector();
        Vector side = fwd.copy();
        side.rotateVector(Math.PI / 2);
        fwd.setMagnitude(Robot.ROBOT_LENGTH / 2.0);
        side.setMagnitude(Robot.ROBOT_WIDTH / 2.0);

        {
            double x1 = t.getX() + fwd.getXComponent() + side.getXComponent();
            double y1 = t.getY() + fwd.getYComponent() + side.getYComponent();
            double x2 = t.getX() + fwd.getXComponent() - side.getXComponent();
            double y2 = t.getY() + fwd.getYComponent() - side.getYComponent();
            double x3 = t.getX() - fwd.getXComponent() - side.getXComponent();
            double y3 = t.getY() - fwd.getYComponent() - side.getYComponent();
            double x4 = t.getX() - fwd.getXComponent() + side.getXComponent();
            double y4 = t.getY() - fwd.getYComponent() + side.getYComponent();
            double[] xPoints = new double[]{x1, x2, x3, x4};
            double[] yPoints = new double[]{y1, y2, y3, y4};
            c.strokePolygon(xPoints, yPoints);
        }

        {
            double x1 = t.getX() + fwd.getXComponent() / 2.0;
            double y1 = t.getY() + fwd.getYComponent() / 2.0;
            double x2 = t.getX() + fwd.getXComponent();
            double y2 = t.getY() + fwd.getYComponent();
            c.strokeLine(x1, y1, x2, y2);
        }
    }

    public static void drawPath(Canvas c, double[][] points) {
        c.strokePolyline(points[0], points[1]);
    }

    private static Pose raycastToFieldEdge(Pose pose) {
        pose = pose.getAsCoordinateSystem(FTCCoordinates.INSTANCE);

        double x = pose.getX();
        double y = pose.getY();

        Vector dir = pose.getHeadingAsUnitVector();
        double dx = dir.getXComponent();
        double dy = dir.getYComponent();

        double minT = Double.POSITIVE_INFINITY;
        double hitX = x;
        double hitY = y;

        // Vertical walls
        if (dx != 0) {
            for (double wallX : new double[]{0, 144}) {
                double t = (wallX - x) / dx;
                if (t > 0) {
                    double yHit = y + t * dy;
                    if (yHit >= 0 && yHit <= 144 && t < minT) {
                        minT = t;
                        hitX = wallX;
                        hitY = yHit;
                    }
                }
            }
        }

        // Horizontal walls
        if (dy != 0) {
            for (double wallY : new double[]{0, 144}) {
                double t = (wallY - y) / dy;
                if (t > 0) {
                    double xHit = x + t * dx;
                    if (xHit >= 0 && xHit <= 144 && t < minT) {
                        minT = t;
                        hitX = xHit;
                        hitY = wallY;
                    }
                }
            }
        }

        return new Pose(hitX, hitY, pose.getHeading());
    }
}
