package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.robot.hardware.TerrorLight;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.util.Profiler;

//im aware i can just call this lightsubsystem but calling it a whole subsystem is kinda doin too much
public class LightControl extends SubsystemBase {

    private final RobotHardware hardware;
    private final Robot robot;
    public static boolean debug = true;

    public void setManualLightColor(TerrorLight.LightColors color)
    {
        hardware.lights.setColor(color);
    }

    public LightControl(RobotHardware hardware, Robot robot) {
        this.hardware = hardware;
        this.robot = robot;
    }


    @Override
    public void periodic()
    {
        try (Profiler.Scope p = Profiler.enter("LightControl2")) {
            if (debug) {
                robot.telemetry.addData("State for the lights", robot.getState());
            }
            switch (robot.robotState) {
                case RESTING:
                    hardware.lights.setColor(TerrorLight.LightColors.PINK);
                    break;

                case INTAKING:
                    hardware.lights.setColor(TerrorLight.LightColors.BLUE);
                    break;

                case TRANSFER:
                    hardware.lights.setColor(TerrorLight.LightColors.ORANGE);
                    break;

                case READY_TO_SHOOT:
                    if (robot.shooter.isTurretInDeadzone()) {
                        hardware.lights.setColor(TerrorLight.LightColors.RED);
                    } else {
                        hardware.lights.setColor(TerrorLight.LightColors.GREEN);
                    }
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
}