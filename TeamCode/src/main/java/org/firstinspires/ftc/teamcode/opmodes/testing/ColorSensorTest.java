package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while (opModeIsActive())
        {
            hardware.rightSensor.update();
            hardware.leftSensor.update();
            hardware.topSensor.update();

//            NormalizedRGBA right = hardware.rightSensor.getNormalizedColors();
            telemetry.addData("color red", hardware.rightSensor.getRed());
            telemetry.addData("color green", hardware.rightSensor.getGreen());
            telemetry.addData("color blue", hardware.rightSensor.getBlue());

            telemetry.addData("right sensor color", hardware.rightSensor.getGreenOrPurple());

//            NormalizedRGBA left = hardware.leftSensor.getNormalizedColors();
            telemetry.addData("left sensor color", hardware.leftSensor.getGreenOrPurple());

//            NormalizedRGBA top = hardware.topSensor.getNormalizedColors();
            telemetry.addData("top sensor color", hardware.topSensor.getGreenOrPurple());

            telemetry.addData("distance", hardware.rightSensor.getDist(DistanceUnit.MM));
            telemetry.addData("distance", hardware.leftSensor.getDist(DistanceUnit.MM));
            telemetry.addData("distance", hardware.topSensor.getDist(DistanceUnit.MM));
            telemetry.update();
        }
    }
}
