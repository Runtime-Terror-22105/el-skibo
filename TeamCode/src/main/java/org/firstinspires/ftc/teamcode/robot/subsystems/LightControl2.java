package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;
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
public class LightControl2 extends SubsystemBase {

    private double blinkColor = TerrorLight.LightColors.OFF.ordinal();
    //
    private boolean isBlinkOn = false;

    private final RobotHardware hardware;
    private final Robot robot;
    private final ElapsedTime time = new ElapsedTime();

    public LightControl2(RobotHardware hardware, Robot robot) {
        this.hardware = hardware;
        this.robot = robot;
        time.reset();
    }


    @Override
    public void periodic()
    {
        //eventually most of these states wont have anything special so a hashmap would make iteasy
        //just do 3 ifstatmeents at most: if(insert special case) if(another special) else(mapcolor)
        switch(robot.getState())
        {
            case RESTING:
                hardware.lights.setColor(TerrorLight.LightColors.PINK);
                break;

            case INTAKING:
                hardware.lights.setColor(TerrorLight.LightColors.BLUE);
                break;

            //i might be misunderstanding
//            case FULL:
//                hardware.lights.setColor(TerrorLight.LightColors.GREEN);
//                break;
            case TRANSFER:
                hardware.lights.setColor(TerrorLight.LightColors.ORANGE);
                break;

            case SHOOTING:
                hardware.lights.setColor(TerrorLight.LightColors.GREEN);

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
