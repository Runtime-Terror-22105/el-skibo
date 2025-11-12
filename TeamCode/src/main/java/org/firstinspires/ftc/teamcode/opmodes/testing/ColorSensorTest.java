package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

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
            NormalizedRGBA right = hardware.rightSensor.getNormalizedColors();
            telemetry.addData("right sensor red", right.red * 255);
            telemetry.addData("right sensor green", right.green * 255);
            telemetry.addData("right sensor blue", right.blue * 255);

            NormalizedRGBA left = hardware.leftSensor.getNormalizedColors();
            telemetry.addData("left sensor red", left.red * 255);
            telemetry.addData("left sensor green", left.green * 255);
            telemetry.addData("left sensor blue", left.blue * 255);

            NormalizedRGBA top = hardware.topSensor.getNormalizedColors();
            telemetry.addData("top sensor red", top.red * 255);
            telemetry.addData("top sensor green", top.green * 255);
            telemetry.addData("top sensor blue", top.blue * 255);
            telemetry.update();
        }
    }
}
