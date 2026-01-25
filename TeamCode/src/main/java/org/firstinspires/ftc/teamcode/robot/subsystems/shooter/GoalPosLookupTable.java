package org.firstinspires.ftc.teamcode.robot.subsystems.shooter;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.VectorCalculator;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.apache.commons.math3.analysis.function.Acos;
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



            new GoalLookupValue(0.3, 0),
            new GoalLookupValue(0.6, 5),
            new GoalLookupValue(0.85, 12),
            new GoalLookupValue(0.92, 10),
            new GoalLookupValue(1.13, 10),
            //Currently these are just guesses
//            new GoalLookupValue(0.233, -12),
//            new GoalLookupValue(0.29, -8),
//            new GoalLookupValue(0.806, 7),
//            new GoalLookupValue(0.91, 12),
//            new GoalLookupValue(1.1, 10),

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

        if (debug) Log.d("GoalPosLookupTable","robot vector" + goalToRobot);
        if (debug) Log.d("GoalPosLookupTable","angle" + angle);

        return angle;
    }

    public Pose2d get(){
        updateDataPoints();
        if (debug) Log.d("GoalPosLookupTable", "our team"+robot.color);

//        double change = GOAL_CHANGE_LUT.get(this.calcAngleWithWall());
        Pose2d newGoalPos = FieldConstants.RED_GOAL_POS.copy();

//        if (change < 0){
//            newGoalPos.y -= Math.abs(change);
//        } else if(change > 0){
//            newGoalPos.x -= Math.abs(change);
//        }

        if (robot.color == Team.BLUE) {
            if (debug) Log.d("GoalPosLookupTable", "before mirror"+newGoalPos);
            newGoalPos = newGoalPos.mirror();
        }




        if (debug) Log.d("GoalPosLookupTable", "old goal pos" + robot.goalPos);
        if (debug) Log.d("GoalPosLookupTable", "new goal pos " + newGoalPos);
        return newGoalPos;

    }
}
