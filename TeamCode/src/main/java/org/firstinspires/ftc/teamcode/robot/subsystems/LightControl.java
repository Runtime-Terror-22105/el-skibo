package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.hardware.TerrorLight;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;
import org.firstinspires.ftc.teamcode.robot.init.StateTag;

import java.util.HashMap;
import java.util.Map;

//im aware i can just call this lightsubsystem but calling it a whole subsystem is kinda doin too much
public class LightControl extends SubsystemBase {

//    private double color = TerrorLight.LightColors.OFF.ordinal();
//
//    private boolean isBlinking = false;

    private final RobotHardware hardware;
    private final Robot robot;
    public LightControl(RobotHardware hardware, Robot robot) {
        this.hardware = hardware;
        this.robot = robot;
    }

//    public void setColor(TerrorLight.LightColors color)
//    {
//        this.color = color.ordinal();
//    }
//
//    public void setColor(double color)
//    {
//        this.color = color;
//    }
//
//    public void startBlinking()
//    {
//        this.isBlinking = true;
//    }
//
//    public void stopBlinking()
//    {
//        this.isBlinking = false;
//    }

    @Override
    public void periodic()
    {
        //eventually most of these states wont have anything special so a hashmap would make iteasy
        //just do 3 ifstatmeents at most: if(insert special case) if(another special) else(mapcolor)
        switch(robot.getState())
        {
            case RESTING:
                hardware.lights.setColor(TerrorLight.LightColors.PINK);
                //todo: RTT morse code shouldnt be too hard
                break;

            case INTAKING:
                break;

            //i might be misunderstanding
            case FULL:
                hardware.lights.setColor(TerrorLight.LightColors.GREEN);
                break;
            case TRANSFER:
                hardware.lights.setColor(TerrorLight.LightColors.ORANGE);
                break;

            case SHOOTING:
                hardware.lights.setColor(TerrorLight.LightColors.GREEN);
                try {
                    wait(100); //note there's a 20 ms loop time from what i rmemeber (idk how diff frc is to ftc)
                    hardware.lights.setColor(TerrorLight.LightColors.OFF);
                } catch (InterruptedException e) {
                    //oh well prob some telemetry work
                }
                break;

            case CLIMBING:
                break;

            case DONE_CLIMB:
                break;
            default:
                hardware.lights.setColor(TerrorLight.LightColors.PINK);
                break;
        }

    }
}
