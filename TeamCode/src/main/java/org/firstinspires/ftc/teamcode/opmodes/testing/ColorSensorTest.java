package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.hardware.sensors.TerrorColorSensor;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@TeleOp(name="Color Sensor Test", group="Testing")
public class ColorSensorTest extends LinearOpMode {
    private final RobotHardware hardware = new RobotHardware();

    @Override
    public void runOpMode() throws InterruptedException {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.AUTO);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while (opModeIsActive())
        {
            TerrorColorSensor left = hardware.colorSensors.getLeftUnsafe();
            TerrorColorSensor right = hardware.colorSensors.getRightUnsafe();
            TerrorColorSensor top = hardware.colorSensors.getTopUnsafe();
            
            right.update();
            left.update();
            top.update();

//            NormalizedRGBA right = right.getNormalizedColors();
            telemetry.addData("t color red", top.getRed());
            telemetry.addData("t color green", top.getGreen());
            telemetry.addData("t color blue", top.getBlue());
            telemetry.addData("r color red ", right.getRed());
            telemetry.addData("r color green ", right.getGreen());
            telemetry.addData("r color blue ", right.getBlue());

            telemetry.addData("right sensor color", right.getBallColor());

//            NormalizedRGBA left = left.getNormalizedColors();
            telemetry.addData("left sensor color", left.getBallColor());

//            NormalizedRGBA top = top.getNormalizedColors();
            telemetry.addData("top sensor color", top.getBallColor());

            telemetry.addData("r distance", right.getDist(DistanceUnit.MM));
            telemetry.addData("l distance", left.getDist(DistanceUnit.MM));
            telemetry.addData("t distance", top.getDist(DistanceUnit.MM));
            telemetry.update();
        }
    }
}
