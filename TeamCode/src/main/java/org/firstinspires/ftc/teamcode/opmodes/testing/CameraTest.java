package org.firstinspires.ftc.teamcode.opmodes.testing;

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
        robot.camera.setAprilTagsEnabled(true);
        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive())
        {
            CommandScheduler.getInstance().run();
            robot.telemetry.addData("seenglyph",robot.camera.getGlyph());
            robot.telemetry.addData("loop times", timer.milliseconds());
            timer.reset();
            robot.telemetry.update();
        }

        robot.close();
    }
}
