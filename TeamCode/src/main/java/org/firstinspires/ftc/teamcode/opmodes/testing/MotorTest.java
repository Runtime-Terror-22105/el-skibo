package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@TeleOp(name = "Motor Test", group = "Testing")
@Config
public class MotorTest extends LinearOpMode {
    // targets
    public static double FRONT_LEFT_POWER = 0;
    public static double FRONT_RIGHT_POWER = 0;
    public static double REAR_LEFT_POWER = 0;
    public static double REAR_RIGHT_POWER = 0;

    private final RobotHardware hardware = new RobotHardware();

    @Override
    public void runOpMode() {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        waitForStart();

        while (opModeIsActive()) {
            if (hardware.allHubs != null) {
                for (LynxModule hub : hardware.allHubs) {
                    hub.clearBulkCache();
                }
            }

            hardware.motorFrontLeft.setPower(FRONT_LEFT_POWER);
            hardware.motorFrontRight.setPower(FRONT_RIGHT_POWER);
            hardware.motorRearLeft.setPower(REAR_LEFT_POWER);
            hardware.motorRearRight.setPower(REAR_RIGHT_POWER);

            hardware.write();
        }
    }
}