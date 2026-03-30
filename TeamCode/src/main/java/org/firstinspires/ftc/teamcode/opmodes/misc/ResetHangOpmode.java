package org.firstinspires.ftc.teamcode.opmodes.misc;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@Config
@TeleOp(name = "Reset Hang", group = "TeleOp")
public class ResetHangOpmode extends LinearOpMode {
    public static volatile double HANG_LOWERING_POWER = 1.0;

    private final RobotHardware hw = new RobotHardware();

    @Override
    public void runOpMode() throws InterruptedException {
        hw.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            for (LynxModule hub : hw.allHubs) {
                hub.clearBulkCache();
            }

            telemetry.addLine("Time to reset hang!");
            telemetry.addLine("Hold the left trigger (LT) to lower the robot.");
            telemetry.addLine("Press (A) to reset the PTO.");
            telemetry.update();

            if (gamepad1.left_trigger > 0.3) {
                hw.spindexer.setPower(-HANG_LOWERING_POWER);
            } else {
                hw.spindexer.setPower(0);
            }

            hw.write();
        }
    }
}
