package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.robot.command.shooter.SpindexerHoming;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.TransferCommand;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@TeleOp
public class TransferTest extends LinearOpMode {
    private final RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();
    @Override
    public void runOpMode() {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        robot.init(hardware, telemetry);
        waitForStart();

        TransferCommand transfer = new TransferCommand(robot.spindexer);
        while (opModeIsActive()) {
            // Manually clear the bulk read cache. Deleting this would be catastrophic b/c stale
            // vals would be used.
            for (LynxModule hub : hardware.allHubs) {
                hub.clearBulkCache();
            }

            if (gamepad1.aWasPressed()) {
                transfer.phase1();
            }
            else if (gamepad1.bWasPressed()) {
                transfer.phase2();
            }
            else if (gamepad1.yWasPressed()) {
                transfer.phase3();
            }
            else if (gamepad1.xWasPressed()) {
                transfer.phase4();
            }
            else if (gamepad1.leftBumperWasPressed()) {
                transfer.phase5();
            }
            hardware.write();
        }

    }
}
