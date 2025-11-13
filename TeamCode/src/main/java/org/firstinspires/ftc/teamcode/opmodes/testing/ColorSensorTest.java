package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
            telemetry.addData("color", hardware.rightSensor.getVersion());


            telemetry.addData("right sensor red", hardware.rightSensor.getRed());
            telemetry.addData("right sensor green",  hardware.rightSensor.getGreen());
            telemetry.addData("right sensor blue", hardware.rightSensor.getBlue());
            telemetry.addData("right sensor color", hardware.rightSensor.getGreenOrPurple());
            telemetry.addData("distance", hardware.rightSensor.getDist(DistanceUnit.MM));

            NormalizedRGBA left = hardware.leftSensor.getNormalizedColors();
            telemetry.addData("left sensor color", hardware.leftSensor.getGreenOrPurple());

            NormalizedRGBA top = hardware.topSensor.getNormalizedColors();
            telemetry.addData("top sensor color", hardware.topSensor.getGreenOrPurple());
            telemetry.update();
        }
    }
}
