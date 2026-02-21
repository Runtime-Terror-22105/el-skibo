package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;

@Config
@TeleOp(name="Encoder Test", group="Testing")
public class EncoderTest extends LinearOpMode {

    double lowestVoltage = 1;
    double highestVoltage = 0;

    private RobotHardware hardware = new RobotHardware();

    @Override
    public void runOpMode() throws InterruptedException {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.AUTO);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        waitForStart();
        while(opModeIsActive())
        {
            double voltage = hardware.hwMap.get(AnalogInput.class, "spindexEncoder").getVoltage();
            if(voltage > highestVoltage)
            {
                highestVoltage = voltage;
            }
            if(voltage < lowestVoltage)
            {
                lowestVoltage = voltage;
            }
            telemetry.addData("lowestVoltage ",String.valueOf(lowestVoltage));
            telemetry.addData("highestVoltage ",String.valueOf(highestVoltage));
            telemetry.addLine("ANALOG ENCODER");
            telemetry.addLine("--------------");
            telemetry.addData("raw voltage (volts)",String.valueOf(voltage));
            telemetry.addData("without offset (degrees)", Math.toDegrees(hardware.spindexerEncoder.getCurrentPositionWithoutOffset()));
            telemetry.addData("with offset (degrees)", Math.toDegrees(hardware.spindexerEncoder.getCurrentPosition()));
            telemetry.addLine();
            telemetry.addLine();
            telemetry.addLine("MOTOR ENCODER");
            telemetry.addLine("--------------");
            telemetry.addData("Position (ticks)", getPositionTicks());
            telemetry.addData("Position (degrees)", Math.toDegrees(getPosition()));

            telemetry.update();
        }
    }

    public double getPositionTicks() {
        return hardware.spindexerMotorEncoder.getCurrentPosition();
    }

    public double getPosition() {
        return SpindexerSubsystem.ticksToRadians(getPositionTicks());
    }
}