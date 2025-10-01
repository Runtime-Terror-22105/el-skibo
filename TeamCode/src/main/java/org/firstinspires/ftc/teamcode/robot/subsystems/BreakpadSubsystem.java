package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.robot.hardware.motors.TerrorServo;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

public class BreakpadSubsystem extends SubsystemBase {
    private final RobotHardware hardware;
    private final TerrorServo breakServo;
    private final double breakPosition = 1;
    private final double freePosition = -1;

    public BreakpadSubsystem(RobotHardware hardware) {
        this.hardware = hardware;
        this.breakServo = hardware.breakPad;
    }

    public void activateBreak()
    {
        this.breakServo.setPosition(breakPosition);
    }

    public void deactivateBreak()
    {
        this.breakServo.setPosition(freePosition);
    }

    @Override
    public void periodic() {
        // TODO: do something with hardware.turret here...
    }
}
