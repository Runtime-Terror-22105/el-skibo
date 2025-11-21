package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
//@Config
public class HangSubsystem extends SubsystemBase {

    private final RobotHardware hardware;

    private boolean ptoEngaged = false;

    public static double engagedPosition = 1;
    public static double disgagePosition = -1;

    public void setPTOState(boolean state)
    {
        ptoEngaged = state;
        hardware.spindexerPTO.setPosition(state ? engagedPosition : disgagePosition);
    }

    public HangSubsystem(RobotHardware hardware) {
        this.hardware = hardware;
    }

    @Override
    public void periodic() {
        if(ptoEngaged)
        {
            hardware.spindexerRotate.setPower(1); //this probably needs review i have 0 clue
        }
    }
}
