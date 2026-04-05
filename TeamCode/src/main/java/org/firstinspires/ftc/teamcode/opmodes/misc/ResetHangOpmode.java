package org.firstinspires.ftc.teamcode.opmodes.misc;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.subsystems.HangSubsystem;

@Config
@TeleOp(name = "Reset Hang", group = "TeleOp")
public class ResetHangOpmode extends LinearOpMode {
    private final RobotHardware hw = new RobotHardware();

    @Override
    public void runOpMode() throws InterruptedException {
        hw.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        hw.pto.setPosition(HangSubsystem.PTO_DISENGAGE_POSITION);
        hw.write();
    }
}
