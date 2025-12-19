package org.firstinspires.ftc.teamcode.opmodes.misc;

import static org.firstinspires.ftc.teamcode.robot.subsystems.HangSubsystem.HANG_SPINDEXER_POWER;
import static org.firstinspires.ftc.teamcode.robot.subsystems.HangSubsystem.PTO_DISENGAGED_POSITION;
import static org.firstinspires.ftc.teamcode.robot.subsystems.HangSubsystem.PTO_ENGAGED_POSITION;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.init.Robot;
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

        boolean ptoActive = true;
        while (opModeIsActive()) {
            for (LynxModule hub : hw.allHubs) {
                hub.clearBulkCache();
            }

            telemetry.addLine("Time to reset hang!");
            telemetry.addLine("Hold the left trigger (LT) to lower the robot.");
            telemetry.addLine("Press (A) to reset the PTO.");
            telemetry.update();

            // note: we do NOT use the robot class bc otherwise disabling the spindexer pid will be annoying
            // and failing to do so could break the robot

            hw.spindexerPTO.setPosition(ptoActive ? PTO_ENGAGED_POSITION : PTO_DISENGAGED_POSITION);

            if (gamepad1.left_trigger > 0.3) {
                hw.spindexerRotate.setPower(-HANG_LOWERING_POWER);
            } else {
                hw.spindexerRotate.setPower(0);
            }

            hw.write();

            if (gamepad1.aWasPressed()) {
                ptoActive = false;
            }
        }
    }
}
