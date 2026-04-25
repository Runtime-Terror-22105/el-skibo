package org.firstinspires.ftc.teamcode.opmodes.testing;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@TeleOp(name="Camera Test", group="Testing")
@Config
public class CameraTest extends LinearOpMode {
    private final RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();

    public static boolean isBlue = false;
    public static boolean isInNearAuto = false;

//    public static volatile boolean streamFrontCamera = false;
//    public static volatile boolean streamBackCamera = true;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.AUTO, RobotHardware.HardwareOptions.CAMERA);
        robot.init(hardware, this);
//        robot.camera.setTeam(Team.RED);
//        if(isBlue)
//        {
//            robot.camera.setTeam(Team.BLUE);
//        }
//        if(isInNearAuto)
//        {
//            robot.camera.setAuto(true,false);
//        }

//        robot.camera.setBallPipelineEnabled(true);
//        robot.camera.setAprilTagsEnabled(false);
        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        // both are initially disabled, so we want to start streaming immediately
//        boolean streamFrontCameraLast = true;
//        boolean streamBackCameraLast = true;
        while (opModeIsActive())
        {
//            // only update the camera streaming state if it has changed, since changing the streaming state is expensive and can cause frame drops
//            if (streamFrontCamera != streamFrontCameraLast) {
//                robot.camera.setFrontCameraStreamEnabled(streamFrontCamera);
//                streamFrontCameraLast = streamFrontCamera;
//            }
//            if (streamBackCamera != streamBackCameraLast) {
//                robot.camera.setBackCameraStreamEnabled(streamBackCamera);
//                streamBackCameraLast = streamBackCamera;
//            }

            CommandScheduler.getInstance().run();
            robot.telemetry.addData("seenglyph",robot.camera.getGlyph());
            robot.telemetry.addData("loop times", timer.milliseconds());
            timer.reset();
            robot.telemetry.update();
        }

        robot.close();

        Log.d("CameraTest", "OpMode finished, camera closed");
    }
}
