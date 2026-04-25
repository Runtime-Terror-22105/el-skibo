package org.firstinspires.ftc.teamcode.robot.command.vision;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.subsystems.vision.CameraSubsystem;

public class SetCameraStreamingCommand extends InstantCommand {
    public static enum CameraType {
        FRONT,
        BACK
    }

    public SetCameraStreamingCommand(CameraSubsystem camera, boolean frontCamera, boolean backCamera) {
//        super(() -> {
//            camera.setFrontCameraStreamEnabled(frontCamera);
//            camera.setBackCameraStreamEnabled(backCamera);
//        });
    }

    public SetCameraStreamingCommand(CameraSubsystem camera, CameraType cameraType, boolean enabled) {
//        super(() -> {
//            if (cameraType == CameraType.FRONT) {
//                camera.setFrontCameraStreamEnabled(enabled);
//            } else {
//                camera.setBackCameraStreamEnabled(enabled);
//            }
//        });
    }
}
