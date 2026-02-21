package org.firstinspires.ftc.teamcode.robot.command.vision;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.subsystems.vision.CameraSubsystem;

public class StopScanningForGlyphsCommand extends InstantCommand {
    public StopScanningForGlyphsCommand(CameraSubsystem camera) {
        super(() -> {
            if (camera != null && camera.getGlyph() != null)
                camera.stopScanningForGlyphs();
        });
    }
}
