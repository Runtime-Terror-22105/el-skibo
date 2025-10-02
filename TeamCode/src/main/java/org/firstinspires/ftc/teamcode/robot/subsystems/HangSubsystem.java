package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

public class HangSubsystem extends SubsystemBase {

    private final RobotHardware hardware;

    private boolean ptoEngaged = false;

    private double engagedPosition = 1;
    private double disgagePosition = -1;

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
