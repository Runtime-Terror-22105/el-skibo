package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@TeleOp(name = "Intake Test", group = "Testing")
@Config
public class Intaketesting extends LinearOpMode {
    // targets
    private RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();



    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL);

        robot.init(hardware, this);

        waitForStart();
        CommandScheduler.getInstance().run();
        while (opModeIsActive()) {
            for (LynxModule hub : hardware.allHubs) {
                hub.clearBulkCache();
            }
            hardware.write();
        }

        robot.close();
    }
}