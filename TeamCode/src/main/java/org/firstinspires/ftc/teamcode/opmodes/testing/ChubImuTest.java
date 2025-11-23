package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@TeleOp
public class ChubImuTest extends LinearOpMode {
    private final RobotHardware hardware = new RobotHardware();

    @Override
    public void runOpMode() throws InterruptedException {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.AUTO, RobotHardware.HardwareOptions.CAMERA);

        waitForStart();

        while (opModeIsActive())
        {
            CommandScheduler.getInstance().run();

            YawPitchRollAngles oadfji = hardware.imu.getRobotYawPitchRollAngles();
            telemetry.addData("Yaw (degrees)", oadfji.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch (degrees)", oadfji.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll (degrees)", oadfji.getRoll(AngleUnit.DEGREES));
            //telemetry.addData("position",robot.camera.getPositionCamera());
//            hardware.fieldCamera

            telemetry.update();
        }
    }
}
