package org.firstinspires.ftc.teamcode.robot.auto.followers;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D;

public class Obstacle {
    public double[]xbounds;

    public double m;
    public double b;

    public double robot_width;
    public final double radius=6.25*Math.pow(2,0.5);
    public double[]ybounds;
    public Obstacle(double[]xbounds,double[]ybounds, double robot_width){
        this.xbounds=xbounds;
        this.ybounds=ybounds;
        this.robot_width=robot_width;

    }


    public boolean intersects(Pose2D init, Pose2D goal){ //coordinates of start and end of the line segment
        double x1=init.x;
        double x2=goal.x;
        double y1=init.y;
        double y2=goal.y;
        this.m=(y2-y1)/(x2-x1);
        this.b=y1-m*x1; // y=mx+b so b=y-mx
        double x_left_intersection=m*(xbounds[0])+b;// the y coordinate of the x-axis left_bound of the obstacle
        double x_right_intersection=m*(xbounds[1])+b;// the y coordinate of the x-axis left_bound of the obstacle
        // if y=mx+b then we can solve for the x-coordinate intersection doing simple algebra so x=(y-b)/m
        double y_left_intersection;
        double y_right_intersection;
        if(m!=0){
            y_left_intersection=(ybounds[0]-b)/m;
            y_right_intersection=(ybounds[0]-b)/m;
        }
        else{
            return false;
        }

        // if it intersects on any sides of the rectangle bounding the obstacle then the path intersects with the obstacle
        if(((xbounds[0]<=x_left_intersection && x_left_intersection<=xbounds[1])||(xbounds[0]<=x_right_intersection && x_right_intersection<=xbounds[1])) && ((ybounds[0]<=y_left_intersection && y_left_intersection<=ybounds[1])||(ybounds[0]<=y_right_intersection && y_right_intersection<=ybounds[1]))){
            return true;
        }
        return false;
        // otherwise there is no y-left or right intersection thing if m=0 since the equation would not even work cuz u cant divide by 0

    }
    public Pose2D[] convertPosetoVertices(double x, double y, double heading){
        double v1_heading=(Math.PI/4)+heading;
        Pose2D v1= new Pose2D(x+radius*Math.cos(v1_heading),y+radius*Math.sin(v1_heading),v1_heading);

        double v2_heading=3*(Math.PI/4)+heading;
        Pose2D v2= new Pose2D(x+radius*Math.cos(v1_heading),y+radius*Math.sin(v1_heading),v1_heading);

        double v3_heading=5*(Math.PI/4)+heading;
        Pose2D v3= new Pose2D(x+radius*Math.cos(v1_heading),y+radius*Math.sin(v1_heading),v1_heading);

        double v4_heading=7*(Math.PI/4)+heading;
        Pose2D v4= new Pose2D(x+radius*Math.cos(v1_heading),y+radius*Math.sin(v1_heading),v1_heading);

        Pose2D[]vertices= new Pose2D[4];
        vertices[0]=v1;
        vertices[1]=v2;
        vertices[2]=v3;
        vertices[3]=v4;

        return vertices;
    }

    public boolean crash_detect(double x_init, double y_init, double x_final, double y_final, double heading){
        Pose2D v_init[]=convertPosetoVertices(x_init,y_init,heading);
        Pose2D v_final[]=convertPosetoVertices(x_final,y_final,heading);


        for(int i=0;i<4;i++){
            Pose2D initial=v_init[i];
            Pose2D goal=v_final[i];
            if(intersects(initial,goal)){
                return true;
            }
        }

        return false;

    }








}
