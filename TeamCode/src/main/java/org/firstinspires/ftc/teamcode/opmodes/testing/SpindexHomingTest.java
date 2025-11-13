package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.robot.command.shooter.SpindexerHoming;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@TeleOp(name="Spindex Homing Tuner", group="Tuning")
public class SpindexHomingTest extends LinearOpMode {

    private final RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();

    @Override
    public void runOpMode() {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        robot.init(hardware, telemetry);
        waitForStart();

        SpindexerHoming homing = new SpindexerHoming(robot.spindexer);
        CommandScheduler.getInstance().schedule(homing);

        while (opModeIsActive()) {
            // Manually clear the bulk read cache. Deleting this would be catastrophic b/c stale
            // vals would be used.
            for (LynxModule hub : hardware.allHubs) {
                hub.clearBulkCache();
            }

            CommandScheduler.getInstance().run();


            hardware.write();


            robot.telemetry.addData("Current Position", robot.spindexer.getPosition());
            robot.telemetry.addData("Switch Pos", homing.limit);
            robot.telemetry.addData("Raw Switch Pos", hardware.spindexerLimitSwitch.getState());
            robot.telemetry.addData("Seen Switch Start", homing.seenSwitchStart);
            robot.telemetry.addData("Switch Start", homing.switchStart);
            robot.telemetry.addData("Switch End", homing.switchEnd);
            robot.telemetry.update();

        }

    }

}
