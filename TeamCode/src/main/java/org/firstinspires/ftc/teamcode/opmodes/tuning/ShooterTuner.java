package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@Config
@TeleOp(name="Shooter Tuner", group="Tuning")
public class ShooterTuner extends LinearOpMode {

    private final RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();

    public static double goalSpeed = 0.0;

    @Override
    public void runOpMode() {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        robot.init(hardware, telemetry);
        waitForStart();

        while (opModeIsActive()) {
            // Manually clear the bulk read cache. Deleting this would be catastrophic b/c stale
            // vals would be used.
            for (LynxModule hub : hardware.allHubs) {
                hub.clearBulkCache();
            }

            robot.shooter.setSpeed(goalSpeed);

//            CommandScheduler.getInstance().run();
            robot.shooter.periodic();

            hardware.write();


            robot.telemetry.addData("Goal Velocity",goalSpeed);
            robot.telemetry.addData("Current velocity",robot.shooter.getVelocityRpm());
            robot.telemetry.update();

        }

    }

}
