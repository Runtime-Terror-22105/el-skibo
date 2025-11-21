package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.hardware.motors.TerrorServo;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

public class TerrorLight implements TerrorWritingDevice{

    public Servo light;

    public LightCommand command;

    private enum LightCommand {
        SET_LIGHT,   // Indicates that a new position needs to be set
        NONE            // No command
    }

    public double color;

    public double lastColor;

    public double tolerance=0.001;
    public TerrorLight(Servo light){
        this.light=light;
    }

    synchronized public void setColor(double color) { // color is taken from 0 to 1
        this.color = color;
        this.command=LightCommand.SET_LIGHT;
    }


    synchronized public void write() {
        if (command.equals(LightCommand.SET_LIGHT) &&
                Math.abs(this.color - this.lastColor) > this.tolerance) {
            this.lastColor = this.color;
            this.light.setPosition(this.color);
        }
        this.command = LightCommand.NONE;
    }

    synchronized public double getColor() {
        return this.light.getPosition();
    }

}
