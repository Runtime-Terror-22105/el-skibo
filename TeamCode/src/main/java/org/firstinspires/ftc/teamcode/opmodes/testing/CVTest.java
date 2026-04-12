package org.firstinspires.ftc.teamcode.opmodes.testing;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.subsystems.vision.CameraSubsystem;

@TeleOp(name=" V Test", group="Testing")
@Config
public class CVTest extends LinearOpMode {
    private final RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();

    public static boolean isRamp = false;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.AUTO, RobotHardware.HardwareOptions.CAMERA);
        robot.init(hardware, this);
        robot.camera.setAprilTagsEnabled(false);
        if(isRamp)
        {
            robot.camera.setCVMode(CameraSubsystem.FRONT_CV_MODE.RAMP);
        }
        else
        {
            robot.camera.setCVMode(CameraSubsystem.FRONT_CV_MODE.FAR);
        }

        waitForStart();

        while (opModeIsActive())
        {
            CommandScheduler.getInstance().run();
            robot.telemetry.update();
        }

        robot.close();
    }
}
