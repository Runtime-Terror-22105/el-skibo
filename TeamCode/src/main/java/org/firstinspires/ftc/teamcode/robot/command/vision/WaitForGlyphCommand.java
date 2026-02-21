package org.firstinspires.ftc.teamcode.robot.command.vision;

import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.robot.subsystems.vision.CameraSubsystem;

public class WaitForGlyphCommand extends WaitUntilCommand {
    public WaitForGlyphCommand(CameraSubsystem camera) {
        super(() -> camera.getGlyph() != null);
    }
}
