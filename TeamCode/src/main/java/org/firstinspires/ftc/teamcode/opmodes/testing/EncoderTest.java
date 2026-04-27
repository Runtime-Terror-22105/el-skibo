package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@Config
@TeleOp(name="Encoder Test", group="Testing")
public class EncoderTest extends LinearOpMode {
    private RobotHardware hardware = new RobotHardware();

    @Override
    public void runOpMode() throws InterruptedException {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.AUTO);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        waitForStart();
        while(opModeIsActive())
        {
            double spindexVoltage = hardware.hwMap.get(AnalogInput.class, "spindexEncoder").getVoltage();
            telemetry.addData("spindex - raw voltage (volts)", String.valueOf(spindexVoltage));
            telemetry.addData("spindex - without offset (degrees)", Math.toDegrees(hardware.spindexerEncoder.getCurrentPositionWithoutOffset()));
            telemetry.addData("spindex - with offset (degrees)", Math.toDegrees(hardware.spindexerEncoder.getCurrentPosition()));

            double motorRearLeftVoltage = hardware.hwMap.get(AnalogInput.class, "rearLeftEncoder").getVoltage();
            telemetry.addData("motor rear left - raw voltage (volts)", String.valueOf(motorRearLeftVoltage));
            telemetry.addData("motor rear left - without offset (degrees)", Math.toDegrees(hardware.motorRearLeftAbsEncoder.getCurrentPositionWithoutOffset()));
            telemetry.addData("motor rear left - with offset (degrees)", Math.toDegrees(hardware.motorRearLeftAbsEncoder.getCurrentPosition()));

            double motorRearRightVoltage = hardware.hwMap.get(AnalogInput.class, "rearRightEncoder").getVoltage();
            telemetry.addData("motor rear right - raw voltage (volts)", String.valueOf(motorRearRightVoltage));
            telemetry.addData("motor rear right - without offset (degrees)", Math.toDegrees(hardware.motorRearRightAbsEncoder.getCurrentPositionWithoutOffset()));
            telemetry.addData("motor rear right - with offset (degrees)", Math.toDegrees(hardware.motorRearRightAbsEncoder.getCurrentPosition()));
            telemetry.update();
        }
    }
}