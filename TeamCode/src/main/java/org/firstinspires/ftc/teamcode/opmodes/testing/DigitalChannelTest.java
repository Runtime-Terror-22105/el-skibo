package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@TeleOp
public class DigitalChannelTest extends LinearOpMode {

    private RobotHardware hardware = new RobotHardware();

    @Override
    public void runOpMode() throws InterruptedException {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.AUTO);
        waitForStart();
        while(opModeIsActive())
        {
            telemetry.addData("spindexerencoderpos",String.valueOf(hardware.spindexerLimitSwitch.getState()));
            telemetry.update();
        }
    }
}
