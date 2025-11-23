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

    private final RobotHardware hardware;
    private final Robot robot;

    public LightControl2(RobotHardware hardware, Robot robot) {
        this.hardware = hardware;
        this.robot = robot;
    }


    @Override
    public void periodic()
    {
        robot.telemetry.addData("State for the lights",robot.getState());
        switch(robot.robotState.RESTING)
        {
            case RESTING:
                hardware.lights.setColor(TerrorLight.LightColors.PINK);
                break;

            case INTAKING:
                hardware.lights.setColor(TerrorLight.LightColors.BLUE);
                break;

            //i might be misunderstanding
//           case FULL:
//                hardware.lights.setColor(TerrorLight.LightColors.GREEN);
//                break;

            case NOT_READY:
                hardware.lights.setColor(TerrorLight.LightColors.RED);
                break;

            case TRANSFER:
                hardware.lights.setColor(TerrorLight.LightColors.ORANGE);
                break;

            case READY_TO_SHOOT:
                hardware.lights.setColor(TerrorLight.LightColors.GREEN);

                break;

            case SHOOTING:
                hardware.lights.setColor(TerrorLight.LightColors.YELLOW);
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
