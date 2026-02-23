package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.robot.hardware.TerrorLight;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

//im aware i can just call this lightsubsystem but calling it a whole subsystem is kinda doin too much
public class LightControl extends SubsystemBase {

//    private double blinkColor = TerrorLight.LightColors.OFF.ordinal();
//
    private boolean isBlinkOn = false;

    private final RobotHardware hardware;
    private final Robot robot;
    private final ElapsedTime time = new ElapsedTime();

    public LightControl(RobotHardware hardware, Robot robot) {
        this.hardware = hardware;
        this.robot = robot;
        time.reset();
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
        if (robot.opMode.opModeInInit()) {
            hardware.lights.setColor(robot.color.equals(Team.RED) ? TerrorLight.LightColors.RED : TerrorLight.LightColors.BLUE);
            return;
        }

        //eventually most of these states wont have anything special so a hashmap would make iteasy
        //just do 3 ifstatmeents at most: if(insert special case) if(another special) else(mapcolor)
        switch(robot.getState())
        {
            case RESTING:
                if(time.time() > 100)
                {
                    isBlinkOn = !isBlinkOn;
                    time.reset();
                }
                if(isBlinkOn)
                {
                    hardware.lights.setColor(TerrorLight.LightColors.PINK);
                }
                else
                {
                    hardware.lights.setColor(TerrorLight.LightColors.OFF);
                }
                //todo: RTT morse code shouldnt be too hard
                break;

            case INTAKING:
                hardware.lights.setColor(TerrorLight.LightColors.BLUE);
                break;

            //i might be misunderstanding
            case READY_TO_SHOOT:
                hardware.lights.setColor(TerrorLight.LightColors.GREEN);
                break;
            case TRANSFER:
                hardware.lights.setColor(TerrorLight.LightColors.ORANGE);
                break;

            case SHOOTING:
                if(time.time() > 100)
                {
                    isBlinkOn = !isBlinkOn;
                    time.reset();
                }
                if(isBlinkOn)
                {
                    hardware.lights.setColor(TerrorLight.LightColors.GREEN);
                }
                else
                {
                    hardware.lights.setColor(TerrorLight.LightColors.OFF);
                }
                break;

            case HANGING_90:
                break;

            case HANGING_FINAL:
                break;


            default:
                if(time.time() > 1000)
                {
                    isBlinkOn = !isBlinkOn;
                    time.reset();
                }
                if(isBlinkOn)
                {
                    hardware.lights.setColor(TerrorLight.LightColors.PINK);
                }
                else
                {
                    hardware.lights.setColor(TerrorLight.LightColors.OFF);

                }
                break;
        }

    }
}
