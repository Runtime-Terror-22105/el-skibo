package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@Config
@TeleOp(name="Shooter Velocity Test", group="Testing")
public class ShooterVelocityTest extends LinearOpMode {

    private final RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();

    public static double power = 0.0;

    @Override
    public void runOpMode() {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        robot.init(hardware, telemetry);
        waitForStart();

        double maxvel=0;

        while (opModeIsActive()) {
            // Manually clear the bulk read cache. Deleting this would be catastrophic b/c stale
            // vals would be used.
            for (LynxModule hub : hardware.allHubs) {
                hub.clearBulkCache();
            }

            hardware.shooterLeft.setPower(power);
            hardware.shooterRight.setPower(power);

            hardware.write();

            maxvel=Math.max(maxvel,robot.shooter.getShooterVelocity());
            robot.telemetry.addData("Current velocity", robot.shooter.getShooterVelocity());
            robot.telemetry.addData("Max velocity so far",maxvel);

            robot.telemetry.update();

        }

    }

}
