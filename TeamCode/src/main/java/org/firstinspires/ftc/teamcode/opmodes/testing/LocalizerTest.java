package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@TeleOp(name="Localizer Test", group="Testing")
public class LocalizerTest extends LinearOpMode {
    private RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.AUTO, RobotHardware.HardwareOptions.PINPOINT);
        robot.init(hardware, telemetry);


        waitForStart();

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();

            robot.telemetry.addData("Raw Pinpoint Position", hardware.pinpoint.getPosition());
            robot.telemetry.addData("Localizer Position", robot.localizer.getCurrentPosition());
            robot.telemetry.update();
        }
    }
}
