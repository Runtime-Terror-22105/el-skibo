package org.firstinspires.ftc.teamcode.math;

import java.util.HashMap;
// SWITCH CASES ARE FOR LAZY PPL AND LOSERS(jk pls dont hurt me)
// best made code(very organized and not messy!!!!!)
/// eat daves hot chicken guys
public class Scurve {
    public double T;

    public final double jm=1.0; // jerk maximum
    public double vs=1.0; // velocity set
    public double as=1.0; // acceleration set

    public double v0; // initial velocity

    public double vh; // midpoint velocity, point of inflection for vel

    public double line_length=0.0; // length of middle section


    // equations
    public upcurve.concave upcave;
    public upcurve.convex upvex;
    public downcurve.convex downvex;
    public downcurve.concave downcave;
    public linear line;

    public double max_withoutline_area=2;  // remember to update this according to system changes

    public double current_position=0;


    public Scurve(double v0, double move_length) {
        // solving the max acceleration to set the system too
        // this is the time

        this.v0 = v0;
        // calculate the parameters for accel and line length
        CalculateParameters(this.v0, move_length); // updates the velocity set speed and possibly the accel set speed
        this.T = 2 * as / jm;
        // these are the curve creations based on the variables
        upcurve up = new upcurve();
        this.upcave = up.new concave();
        this.upvex = up.new convex();
        downcurve down = new downcurve();
        this.line = new linear();
        this.downvex = down.new convex();
        this.downcave = down.new concave();
        this.line_length = (move_length - (max_withoutline_area)) / this.vs;
        this.vh=vs/2;
    }

    public void CalculateParameters(double v0, double deltaPosition){
        if(deltaPosition-(max_withoutline_area)>0){ // has the line segment
            double T=2*this.as/this.jm;
            upcurve up=new upcurve();
            upcurve.concave upc = up.new concave();
            upcurve.convex upv = up.new convex();
            this.vs=upv.getVelocity(T);
            this.line_length=(deltaPosition-(max_withoutline_area))/vs;
        }
        else{ // no line segment
            solveAccel(deltaPosition);
            double T=2*this.as/this.jm;
            upcurve up=new upcurve();
            upcurve.concave upc = up.new concave();
            upcurve.convex upv = up.new convex();
            this.vs=upv.getVelocity(T);
            this.line_length=0;
        }
    }

    public double getTime(double Position){ // from position getting time
        // all the positions at the end of each segment interval(bounds basically)
        double a=0;
        double b=getPosition(T/2);
        double c=getPosition(T);
        double d=getPosition(T+line_length);
        double e=getPosition(1.5*T+line_length);
        double f=getPosition(2*T+line_length);
        this.current_position=Position;
        if(a<=Position && Position<=b){ //works
            return this.upcave.getTime(Position);
        }
        else if(b<=Position && Position<=c){ //works
            return this.upvex.getTime(Position-b);
        }
        else if(c<=Position && Position<=d){ //works
            return this.line.getTime(Position-c);
        }
        else if(d<=Position && Position<=e){
            return this.downvex.getTime(Position- d);
        }
        else if(e<=Position && Position<=f){
            return this.downcave.getTime(Position-e);
        }
        return 0.0;
    }

    public HashMap<String,Double> getParameters(){
        HashMap<String,Double>parameters= new HashMap<>();
        parameters.put("JerkMax",this.jm);
        parameters.put("Accel_set",this.as);
        parameters.put("v0",this.v0);
        parameters.put("line_length",this.line_length);
        parameters.put("vs",this.vs);
        return parameters;
    }


    public double getVelocity(double t){
        if(0<=t && t<=T/2){
            return upcave.getVelocity(t);
        }
        else if(T/2<=t && t<=T){
            return upvex.getVelocity(t);
        }
        // ADD the linear part
        else if(T<=t && t<=T+line_length){
            return line.getVelocity();
        }
        else if(T+line_length<=t && t<=T*1.5+line_length){
            return downvex.getVelocity(t);
        }
        else if(T*1.5+line_length<=t && t<=2*T+line_length){
            return downcave.getVelocity(t);
        }
        return 0.0;
    }

    public double getPosition(double t){ // this is only relative
        if(0<=t && t<=T/2){
            return upcave.getPosition(t);
        }
        else if(T/2<=t && t<=T){
            return upcave.getPosition(T/2)+(upvex.getPosition(t)- upvex.getPosition(T/2));
        }
        else if(T<=t && t<=T+line_length){
            return upcave.getPosition(T/2)+(upvex.getPosition(T)- upvex.getPosition(T/2))+line.getPosition(t);

        }
        else if(T+line_length<=t && t<=T*1.5+line_length){
            return upcave.getPosition(T/2)+(upvex.getPosition(T)- upvex.getPosition(T/2))+(line.getPosition(T+line_length))+(downvex.getPosition(t)- downvex.getPosition(T+line_length));

        }
        else if(T*1.5+line_length<=t && t<=2*T+line_length){
            return upcave.getPosition(T/2)+(upvex.getPosition(T)- upvex.getPosition(T/2))+(line.getPosition(T+line_length))+(downvex.getPosition(T*1.5+line_length)- downvex.getPosition(T+line_length))+(downcave.getPosition(t)- downcave.getPosition(T*1.5+line_length));
        }
        return 0.0; //what weird times are you plugging in even lol
    }

    public void solveAccel(double Position){
        double a=+(6.5/(3*Math.pow(jm,2)))-(7/(6*jm));
        double b=0;
        double c=2*v0/(jm);
        double d=-Position/2;
        CubicEquation cform= new CubicEquation(a,b,c,d);
        this.as=cform.solveCubic();
    }

    public double findsingleroot(double a,double b,double c,double d){
        CubicEquation equation=new CubicEquation(a,b,c,d);
        double correct_root=0;
        double error=Double.MAX_VALUE;
        for(int i=-3;i<4;i++){
            double root=equation.findRoot(i);
            if(Math.abs(getPosition(root)-current_position)<=error){
                correct_root=root;
            }
        }
        return correct_root;
    }








    class upcurve
    {
        class concave{
            concave(){
                vh=getVelocity(T/2);
                vs=(2*vh-v0);

            }
            public double getVelocity(double t){// v(t)=v0+jm*t^2/2
                return v0+((jm)*Math.pow(t,2))/2;
            }
            public double getPosition(double t){
                return v0*t+(1.0/3.0)*(((jm)*Math.pow(t,3))/2.0);
            }

            public double getTime(double position){
                double a=jm/6;
                double b=0;
                double c=v0;
                double d=-position;
                return findsingleroot(a,b,c,d);
            }

        }

        class convex{
            public double getVelocity(double t){ // v(t)=vh+as*(t-T/2)-((jm*(t-T/2))/2)
                double t_shift=t-T/2;
//                double vh=(v0+vs)/2;
                return vh+as*(t_shift)-((jm*Math.pow(t_shift,2))/2.0);
            }
            public double getPosition(double t){ // CHECK THIS STUFF AND THIS CONVEX UP CURVE IDK ABOUT THE MATH.POW divided by 2
                double t_shift=t-T/2;
//                double vh=(v0+vs)/2;
                return vh*t_shift+(1.0/2.0)*as*(Math.pow(t_shift,2))-((1.0/6.0)*(jm*Math.pow(t_shift,3)));
//                return vh*t_shift+(1.0/2.0)*as*(Math.pow(t_shift,2))-((1.0/4.0)*(jm*Math.pow(t_shift,2)));
            }
            public double getTime(double position){
                double a=-1/(jm*6);
                double b=(T/4*jm+as/2);
                double c=-(Math.pow(T,2)/(8*jm))-(as*T/2)+vh;
                double d=((Math.pow(T,3)/(48*jm))+((as*Math.pow(T,2))/8)-(vh*T)/2)+getPosition(T/2)-position;
                return findsingleroot(a,b,c,d);
            }
        }
    }




    class downcurve
    {

        class convex{
            public double getVelocity(double t){ // v(t)=vs-(as/T)*Math.pow(t-T-line_length,2)
                double t_shift=t-T-line_length;
                return vs-(as/T)*Math.pow(t_shift,2);
            }
            public double getPosition(double t){
                double t_shift=t-T-line_length;
                return vs*t_shift-(as/T)*(1.0/3.0)*Math.pow(t_shift,3);
            }
            public double getTime(double position){
                double a=-as/(3*T);
                double b=(as/T)*(T+line_length);
                double c=(-as/T)*Math.pow(T+line_length,2)+vs;
                double d=(as/(3*T))*Math.pow(T+line_length,3)-vs*(T+line_length)+(getPosition(T+line_length)-position);
                return findsingleroot(a,b,c,d);
            }
        }
        class concave{
            public double getVelocity(double t){ // v(t)=vh-as*(t-1.5*T-line_length)+(as/T)*Math.pow(t-1.5*T-line_length,2)
                double t_shift=(t-1.5*T-line_length);
                return vh-as*t_shift+(as/T)*Math.pow(t_shift,2);
            }
            public double getPosition(double t){
//                double vh=(v0+vs)/2;
                double t_shift=(t-1.5*T-line_length);
                return vh*t_shift-0.5*as*Math.pow(t_shift,2)+(as/T)*(1.0/3.0)*Math.pow(t_shift,3);
            }
            public double getTime(double position){
                double a=as/(3*T);
                double b=(-as/T)*(1.5*T+line_length)-0.5*as;
                double c=(as/T)*Math.pow(1.5*T+line_length,2)+as*(1.5*T+line_length)+vh;
                double d=(-as/(3*T))*Math.pow(1.5*T+line_length,3)-0.5*as*Math.pow(1.5*T+line_length,2)-vh*(1.5*T+line_length)-position+getPosition(1.5*T+line_length);
                return findsingleroot(a,b,c,d);
            }
        }
    }

    class linear{
        public double getVelocity(){return vs;}
        public double getPosition(double t){
            return vs*(t-T);
        }

        public double getTime(double position){
            return T+position/vs-getPosition(T);
        }
    }


}
