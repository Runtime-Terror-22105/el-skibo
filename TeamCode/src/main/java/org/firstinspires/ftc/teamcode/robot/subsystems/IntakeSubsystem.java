package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

public class IntakeSubsystem extends SubsystemBase {
    private final RobotHardware hardware;

    public IntakeSubsystem(RobotHardware hardware) {
        this.hardware = hardware;
    }

    @Override
    public void periodic() {
        // TODO: do something with hardware.intake here...
    }
}
