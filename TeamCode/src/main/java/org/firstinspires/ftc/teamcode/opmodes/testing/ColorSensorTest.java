package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@TeleOp
public class ColorSensorTest extends LinearOpMode {
    private final RobotHardware hardware = new RobotHardware();

    @Override
    public void runOpMode() throws InterruptedException {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.AUTO);

        waitForStart();
        while (opModeIsActive())
        {
            telemetry.addData("right sensor red", hardware.rightSensor.red());
            telemetry.addData("right sensor green", hardware.rightSensor.green());
            telemetry.addData("right sensor blue", hardware.rightSensor.blue());

        }
    }
}
