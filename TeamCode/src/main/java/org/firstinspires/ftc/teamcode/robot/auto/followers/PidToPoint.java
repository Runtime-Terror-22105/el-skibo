package org.firstinspires.ftc.teamcode.robot.auto.followers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.controllers.PidController;

@Config
public class PidToPoint {
    public static boolean isEnabled = true;

    public static double REACHED_SLOWDOWN = 1;

    // The threshold for how small the error (in degrees) has to be to use the extra aggressive pid
    public static double HEADING_PID_AGGRESSIVE_THRESHOLD = 1;

    public static PidController.PidCoefficients xCoeff = new PidController.PidCoefficients(0.007, 0, 0.0012, 0.05);
    public static PidController.PidCoefficients yCoeff = new PidController.PidCoefficients(0.013, 0, 0.0015, 0.05);

    // for small turns, use a much more aggressive PID
//    public static PidController.PidCoefficients smallHCoeff = new PidController.PidCoefficients(3.9, 0, 0.22);
    public static PidController.PidCoefficients largeHCoeff = new PidController.PidCoefficients(1.45, 0, 0.08);

    private Pose2d goal;
    private final PidController xController;
    private final PidController yController;
    private final PidController hController;

    public double xTemp;
    public double yTemp;

    /**
     * <p>The amount of milliseconds that the robot needs to be at its destination for it to count
     * as "reaching" its destination.</p>
     * <p>Set this to 0 to disable it.</p>
     */
    private double reachedTime;

    private double lastReachedTime = 0;
    private Pose2d powers = new Pose2d();
    private double speed;

    private boolean reached;

    public PidToPoint() {
        this(
                new Pose2d(0,0,0),
                new Pose2d(0,0,0),
                100
        );
    }

    /**
     * Creates a new pid to point follower.
     * @param goalPoint The goal point to target.
     * @param tolerances The tolerances for the pids.
     * @param reachedTime The amount of time in tolerance before we say we reached (set to 0 to disable).
     */
    public PidToPoint(@NonNull Pose2d goalPoint, @NonNull Pose2d tolerances, double reachedTime) {
        xController = new PidController(xCoeff);
        yController = new PidController(yCoeff);
        hController = new PidController(largeHCoeff);
        this.xController.setTargetPosition(0);
        this.yController.setTargetPosition(0);
        this.hController.setTargetPosition(goalPoint.heading);
        this.setGoal(goalPoint, tolerances, reachedTime);
        this.reached = false;
    }

    public boolean isReached() {
        return this.reached;
    }

    @NonNull
    private Pose2d calculatePidPowers(@NonNull Pose2d currentPos) {
        // Compute the error in global (field-centric) coordinates
        double dx = goal.x - currentPos.x;
        double dy = goal.y - currentPos.y;

        // Rotate the positional error into the robot's reference frame
        double heading = currentPos.heading;
        double cos = Math.cos(-heading);
        double sin = Math.sin(-heading);

        double robotX = dx * cos - dy * sin; // Forward error
        double robotY = dx * sin + dy * cos; // Strafe error

        // Pass robot-relative error into PID controllers
        double xPower = xController.calculatePower(robotX);
        double yPower = yController.calculatePower(robotY);
        double hPower = hController.calculatePower(heading, true);

//        Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
//        telemetry.addData("Robot X", currentPos.x * 0.03937008);
//        telemetry.addData("Robot Y", currentPos.y * 0.03937008);
//        telemetry.addData("Robot Heading", currentPos.heading);
//        telemetry.addData("Goal X", goal.x * 0.03937008);
//        telemetry.addData("Goal Y", goal.y * 0.03937008);
//        telemetry.addData("Goal Heading", goal.heading);
//        telemetry.addData("Robot X Error", robotX * 0.03937008);
//        telemetry.addData("Robot Y Error", robotY * 0.03937008);
//        telemetry.addData("Robot Heading Error", goal.heading - heading);
//        telemetry.addData("X Power", xPower);
//        telemetry.addData("Y Power", yPower);
//        telemetry.addData("H Power", hPower);
//        telemetry.update();

        return isEnabled ? new Pose2d(xPower, yPower, -hPower) : new Pose2d(0, 0, 0);
    }

    public boolean atTargetPosition(@NonNull Pose2d currentPos) {
        return xController.atTargetPosition(xController.getLastError())
                && yController.atTargetPosition(yController.getLastError())
                && hController.atTargetPosition(currentPos.heading);
    }

    public boolean calculate(@NonNull Pose2d currentPos) {
        this.powers = calculatePidPowers(currentPos);

        if (this.atTargetPosition(currentPos)) {
            if (this.reachedTime <= 0) {
//                this.powers = new Pose2d(0, 0, 0);
                this.reached = true;
                return reached;
            }

            if (this.lastReachedTime == 0) {
                // we just reached our target
                this.lastReachedTime = System.currentTimeMillis();
            } else if (System.currentTimeMillis() - this.lastReachedTime >= this.reachedTime) {
                // we've spent the required amount of time at the destination
//                this.powers = new Pose2d(0, 0, 0);
                this.powers.x *= REACHED_SLOWDOWN;
                this.powers.y *= REACHED_SLOWDOWN;

                this.reached = true;
                return reached;
            }
        } else {
            // we haven't reached the destination yet
            this.lastReachedTime = 0;
        }

        this.reached = false;
        return reached;
    }

    public Pose2d getPowers() {
        // pinpoint changed stuff and im lazy so this exists
        return new Pose2d(powers.y, -powers.x, powers.heading).mult(speed);
    }

    public void setGoalPoint(@NonNull Pose2d goalPoint) {
        this.goal = goalPoint;
        this.hController.setTargetPosition(goalPoint.heading);
    }

    public void setGoal(@NonNull Pose2d goalPoint, @NonNull Pose2d tolerances, double reachedTime) {
        this.setGoal(goalPoint, tolerances, 1.0, reachedTime);
    }

    public void setGoal(@NonNull Pose2d goalPoint, @NonNull Pose2d tolerances, double speed, double reachedTime) {
        setGoalPoint(goalPoint);

        this.speed = speed;

        this.xController.setTolerance(tolerances.x);
        this.yController.setTolerance(tolerances.y);
        this.hController.setTolerance(tolerances.heading);

        this.reachedTime = reachedTime;
    }

    public Pose2d getGoal() {
        return goal;
    }

    public Pose2d getTolerances() {
        return new Pose2d(xController.getTolerance(), yController.getTolerance(), hController.getTolerance());
    }

    public double getReachedTime() {
        return this.reachedTime;
    }

    public Pose2d getError() {
        return new Pose2d(
                xController.getLastError(),
                yController.getLastError(),
                hController.getLastError()
        );
    }

    public void resetMovingPowers() {
        this.powers = new Pose2d();
    }
}
