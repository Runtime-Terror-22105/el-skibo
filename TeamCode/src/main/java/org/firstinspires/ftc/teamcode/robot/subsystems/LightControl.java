package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.hardware.TerrorLight;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

import java.util.HashMap;
import java.util.Map;

//im aware i can just call this lightsubsystem but calling it a whole subsystem is kinda doin too much
public class LightControl extends SubsystemBase {

    private double color = TerrorLight.LightColors.OFF.ordinal();

    private boolean isBlinking = false;

    private final RobotHardware hardware;
    public LightControl(RobotHardware hardware) {
        this.hardware = hardware;
    }

    public void setColor(TerrorLight.LightColors color)
    {
        this.color = color.ordinal();
    }

    public void setColor(double color)
    {
        this.color = color;
    }

    public void startBlinking()
    {
        this.isBlinking = true;
    }

    public void stopBlinking()
    {
        this.isBlinking = false;
    }

    @Override
    public void periodic()
    {
        if(isBlinking)
        {
            setColor(this.color);
            try {
                wait(100); //note there's a 20 ms loop time from what i rmemeber (idk how diff frc is to ftc)
                setColor(TerrorLight.LightColors.OFF.ordinal());
            } catch (InterruptedException e) {
                //oh well
            }
            return;
        }
        hardware.lights.setColor(this.color);
    }
}
