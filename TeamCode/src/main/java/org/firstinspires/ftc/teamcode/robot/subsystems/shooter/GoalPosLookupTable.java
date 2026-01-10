package org.firstinspires.ftc.teamcode.robot.subsystems.shooter;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.VectorCalculator;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.apache.commons.math3.analysis.function.Acos;
import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.InterpLUTSafe;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.init.Robot;

@Config
public class GoalPosLookupTable {
    public static double Y_OFFSET = 3.0;
    public static double X_OFFSET = 6.0;

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

            //Currently these are just guesses
            new GoalLookupValue(0, -6),
            new GoalLookupValue((1D/4D * Math.PI), 0),
            new GoalLookupValue((1D/2D * Math.PI), 6)
//            new GoalLookupValue(-Math.PI/2, -12),
//            new GoalLookupValue(-0.94, -12),
//            new GoalLookupValue(-0.9, 0),
//            new GoalLookupValue(-Math.PI/4, 0),
//            new GoalLookupValue(-0.6, 1.67),
//            new GoalLookupValue(-0.5, 4.67),
//            new GoalLookupValue(-0.1, 9.41),
//            new GoalLookupValue(0.25, 9.41),
    };

    private static void updateDataPoints() {
        DATA_POINTS[0].pointChange = -Y_OFFSET;
        DATA_POINTS[1].pointChange = 0;
        DATA_POINTS[2].pointChange = X_OFFSET;

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
    private double calcAngleWithWall(){
        Pose robotPose = robot.follower.getPose();
        if (Team.RED.equals(robot.color)) {
            robotPose = robotPose.mirror();
        }

        Vector goalToRobot = new Vector(new Pose(robotPose.getX(), robotPose.getY()-144D));

        double angle = Math.abs(Angle.angleWrap(goalToRobot.getTheta()));
        if (angle > ((1D/2D)*Math.PI)){
            angle = (1D/4D)*Math.PI;
        }

        Log.d("goalPos","robot vector" + goalToRobot);
        Log.d("goalPos","angle" + angle);

        return angle;
    }

    public Pose2d get(){
        updateDataPoints();

        double change = GOAL_CHANGE_LUT.get(this.calcAngleWithWall());
        Pose2d newGoalPos = robot.goalPos.copy();
        if (change < 0){
            newGoalPos.y -= Math.abs(change);
        } else if(change > 0){
            if (robot.color == Team.BLUE) {
                newGoalPos.x += Math.abs(change);
            } else {
                newGoalPos.x -= Math.abs(change);
            }
        }

        Log.d("goalPos", "old goal pos" + robot.goalPos);
        Log.d("goalPos", "new goal pos " + newGoalPos);
        return newGoalPos;

    }
}
