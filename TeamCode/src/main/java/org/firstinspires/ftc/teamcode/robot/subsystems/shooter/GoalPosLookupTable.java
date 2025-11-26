package org.firstinspires.ftc.teamcode.robot.subsystems.shooter;

import com.pedropathing.VectorCalculator;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.seattlesolvers.solverslib.util.InterpLUT;


import org.apache.commons.math3.analysis.function.Acos;
import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.init.Robot;

public class GoalPosLookupTable {
    private final Robot robot;
    public Pose2d ogGoalPoint;

    private InterpLUT GOAL_CHANGE_LUT;
    public static class GoalLookupValue {
        public double angle;
        public double pointChange;

        public GoalLookupValue(double angle, double change) {
            this.angle = angle;
            this.pointChange = change;
        }
    }

    public static GoalLookupValue[] DATA_POINTS = new GoalLookupValue[]{
            //NEGATIVE VALUE MEANS CHANGE IN X
            //POSITIVE VALUE MEANS CHANGE IN Y

            //Currently these are just guesses
            new GoalLookupValue((1D/4D * Math.PI), 0),
            new GoalLookupValue(0, -2),
            new GoalLookupValue((1D/2D * Math.PI), 2)
    };



    public GoalPosLookupTable(Robot robot){
        this.robot = robot;
        this.ogGoalPoint = robot.goalPos;
    }
    private double calcAngleWithWall(){
        Vector wall;
        Vector goalToRobot;
        Pose robotPose = robot.follower.getPose();
        if (this.robot.color == Team.RED){
            wall = new Vector(144, Math.PI);
            goalToRobot = new Vector(new Pose(robotPose.getX()-144.0, robotPose.getY()));

        }
        else {
            wall = new Vector(144, 0);
            goalToRobot = new Vector(new Pose(robotPose.getX()-144.0, robotPose.getY()-144.0));
        }

        double angle = Math.acos(wall.dot(goalToRobot)/(wall.getMagnitude()*goalToRobot.getMagnitude()));
        return angle;
    }

    public Pose2d get(){
        GOAL_CHANGE_LUT = new InterpLUT();
        for (GoalLookupValue dataPoint : DATA_POINTS) {
            GOAL_CHANGE_LUT.add(dataPoint.angle, dataPoint.pointChange);
        }
        GOAL_CHANGE_LUT.createLUT();

        double change = GOAL_CHANGE_LUT.get(this.calcAngleWithWall());
        Pose2d newGoalPos = robot.goalPos;
        if (change < 0){
            newGoalPos.x -= Math.abs(change);

        }
        else if(change > 0){
            if (robot.color == Team.BLUE) newGoalPos.y += Math.abs(change);
            else newGoalPos.y -= Math.abs(change);

        }
        return newGoalPos;

    }
}
