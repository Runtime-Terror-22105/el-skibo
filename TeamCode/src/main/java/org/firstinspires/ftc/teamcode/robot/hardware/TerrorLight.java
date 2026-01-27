package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.hardware.motors.TerrorServo;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

public class TerrorLight implements TerrorWritingDevice{

    public Servo light;

    public LightCommand command=LightCommand.NONE;

    public enum LightColors {
        // OFF to WHITE are all includded in GoBilda's diagram, others are custom
        // OFF to RED is just all OFF
        // RED to VIOLET goes through all the hues of colors (todo: maybe interpolate this in future)
        // VIOLET to WHITE is just all white
        OFF(0.0),
        RED(0.29),
        ORANGE(0.333),
        YELLOW(0.388),
        SAGE(0.444),
        GREEN(0.500),
        AZURE(0.555),
        BLUE(0.611),
        INDIGO(0.666),
        VIOLET(0.722),
        WHITE(1.0),

        PINK(0.7),
        ;

        private final double servoPosition;

        LightColors(double servoPosition) {
            this.servoPosition = servoPosition;
        }
    }

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

    synchronized public void setColor(LightColors color) { // color is taken from 0 to 1
        this.color = color.servoPosition;
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
