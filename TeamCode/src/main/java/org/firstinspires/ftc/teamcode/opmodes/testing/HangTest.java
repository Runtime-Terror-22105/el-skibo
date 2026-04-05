package org.firstinspires.ftc.teamcode.opmodes.testing;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;

@TeleOp(name="Hang Test", group="Testing")
@Config
public class HangTest extends LinearOpMode {
    private final RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();

    public static double lowPower = -0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        robot.init(hardware, this);

        while(opModeInInit())
        {
            robot.hang.setPTOEngagement(true);
//            hardware.motorRearRight.setPower(lowPower);
//            hardware.motorRearLeft.setPower(lowPower);
//            CommandScheduler.getInstance().run();
//            hardware.write();
        }

        waitForStart();

        while (opModeIsActive())
        {
            robot.robotState = RobotState.HANGING;
            CommandScheduler.getInstance().run();
            hardware.write();
        }

        robot.hang.setPTOEngagement(false);

        robot.close();
    }


}
