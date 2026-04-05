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
import org.firstinspires.ftc.teamcode.robot.subsystems.HangSubsystem;

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
            robot.robotState = RobotState.HANG_INIT;
            CommandScheduler.getInstance().run();
            hardware.write();
        }

        waitForStart();

        while (opModeIsActive())
        {
            robot.robotState = RobotState.HANGING;
            CommandScheduler.getInstance().run();
            hardware.write();
        }

        hardware.pto.setPosition(HangSubsystem.PTO_DISENGAGE_POSITION);
        hardware.write();

        robot.close();
    }


}
