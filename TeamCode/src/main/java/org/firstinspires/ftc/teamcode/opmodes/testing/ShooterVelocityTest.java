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

    public static double power = 0.0;

    @Override
    public void runOpMode() {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
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

            telemetry.addData("Current velocity (ticks/sec)", hardware.shooterEncoder.getVelocity());

            // I'm pretty sure we're using this motor? https://www.gobilda.com/5202-series-yellow-jacket-planetary-gear-motor-5-2-1-ratio-1150-rpm-3-3-5v-encoder/
            telemetry.addData("Current velocity (rpm)", hardware.shooterEncoder.getVelocity() * 60 / 145.1); // 145.1 ticks per revolution
            maxvel=Math.max(maxvel, hardware.shooterEncoder.getVelocity() * 60 / 145.1);
            telemetry.addData("Max Velocity(rpm)", maxvel); // 145.1 ticks per revolution
            telemetry.update();

            telemetry.update();

        }

    }

}
