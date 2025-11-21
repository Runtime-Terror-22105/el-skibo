package org.firstinspires.ftc.teamcode.robot.subsystems;

import static org.firstinspires.ftc.teamcode.robot.init.StateTag.CLIMB;
import static org.firstinspires.ftc.teamcode.robot.init.StateTag.FLYWHEEL_OFF;
import static org.firstinspires.ftc.teamcode.robot.init.StateTag.FLYWHEEL_ON;
import static org.firstinspires.ftc.teamcode.robot.init.StateTag.FUNNEL_READY;
import static org.firstinspires.ftc.teamcode.robot.init.StateTag.INTAKE_DOWN;
import static org.firstinspires.ftc.teamcode.robot.init.StateTag.INTAKE_FORWARD;
import static org.firstinspires.ftc.teamcode.robot.init.StateTag.INTAKE_OFF;
import static org.firstinspires.ftc.teamcode.robot.init.StateTag.INTAKE_REVERSE;
import static org.firstinspires.ftc.teamcode.robot.init.StateTag.INTAKE_UP;

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
        switch(robot.getState())
        {
            case RESTING:
                hardware.lights.setColor(TerrorLight.LightColors.PINK);
                //todo: RTT morse code shouldnt be too hard
                break;

            case INTAKING:
                break;

            case FULL:
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
        }

    }
}
