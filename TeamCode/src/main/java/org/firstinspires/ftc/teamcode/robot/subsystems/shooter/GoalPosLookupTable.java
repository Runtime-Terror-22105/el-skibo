package org.firstinspires.ftc.teamcode.robot.subsystems.shooter;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.InterpLUTSafe;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.init.Robot;

@Config
public class GoalPosLookupTable {
    public static double Y_OFFSET = 4.0;
    public static double X_OFFSET = 6.0;

    public static boolean debug;
    public static boolean telemetry = true;

    private final Robot robot;
    public Pose2d ogGoalPoint;

    private static InterpLUTSafe GOAL_CHANGE_LUT;
    public static class GoalLookupValue {
        public double angle;
        public double pointChange;

        public GoalLookupValue(double angle, double change) {
            this.angle = angle;
            this.pointChange = change;
        }
    }

    public static GoalLookupValue[] DATA_POINTS = new GoalLookupValue[]{
            //NEGATIVE VALUE MEANS CHANGE IN Y
            //POSITIVE VALUE MEANS CHANGE IN X



            new GoalLookupValue(0.3, 9),
            new GoalLookupValue(0.6, 3),
            new GoalLookupValue(0.7, 5),
            new GoalLookupValue(0.85, 3),
            new GoalLookupValue(0.92, 2.5),
            new GoalLookupValue(1.01, 8),
            new GoalLookupValue(1.09, 5),
            new GoalLookupValue(1.18, 7),
            new GoalLookupValue(1.2, 11.5),
            new GoalLookupValue(1.28, 9.5),
    };

    private static void updateDataPoints() {

        GOAL_CHANGE_LUT = new InterpLUTSafe();
        for (GoalLookupValue dataPoint : DATA_POINTS) {
            GOAL_CHANGE_LUT.add(dataPoint.angle, dataPoint.pointChange);
        }
        GOAL_CHANGE_LUT.createLUT();
    }

    public GoalPosLookupTable(Robot robot){
        this.robot = robot;
        this.ogGoalPoint = this.robot.goalPos;
    }

    // Note: Returned angle is returned as if the robot color is always blue.
    private double calcAngleWithWall(Pose robotPose){
        if (Team.RED.equals(robot.color)) {
            robotPose = robotPose.mirror();
        }

        Vector goalToRobot = new Vector(new Pose(robotPose.getX(), robotPose.getY()-144D));

        double angle = Math.abs(Angle.angleWrap(goalToRobot.getTheta()));
        if (angle > ((1D/2D)*Math.PI)){
            angle = (1D/4D)*Math.PI;
        }

        if (debug) Log.d("GoalPosLookupTable","robot vector" + goalToRobot);
        if (debug) Log.d("GoalPosLookupTable","angle" + angle);
        if (telemetry) robot.telemetry.addData("robot angle", angle);

        return angle;
    }

    public Pose2d getForPose(Pose robotPose){
        updateDataPoints();
        if (debug) Log.d("GoalPosLookupTable", "our team"+robot.color);

        double change = GOAL_CHANGE_LUT.get(this.calcAngleWithWall(robotPose));
        Pose2d newGoalPos = FieldConstants.RED_GOAL_POS.copy();

        if (change < 0){
            newGoalPos.y -= Math.abs(change);
        } else if(change > 0){
            newGoalPos.x -= Math.abs(change);
        }

        if (robot.color == Team.BLUE) {
            if (debug) Log.d("GoalPosLookupTable", "before mirror"+newGoalPos);
            newGoalPos = newGoalPos.mirror();
        }




        if (debug) Log.d("GoalPosLookupTable", "old goal pos" + robot.goalPos);
        if (debug) Log.d("GoalPosLookupTable", "new goal pos " + newGoalPos);
        return newGoalPos;

    }

    public Pose2d get() {
        return this.getForPose(this.robot.follower.getPose());
    }
}
