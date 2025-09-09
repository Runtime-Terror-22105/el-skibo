package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

public class SpindexerSubsystem extends SubsystemBase {
    private final RobotHardware hardware;

    public SpindexerSubsystem(RobotHardware hardware) {
        this.hardware = hardware;
    }

    @Override
    public void periodic() {
        // TODO: do something with hardware.spindexer here...
    }
}
