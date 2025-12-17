package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.robot.command.DriveCommand;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem;

@Config
@TeleOp(name="aadit yapping abt smth idk", group="Testing")
public class aadittalkingabtsmthidk extends LinearOpMode {

    private final RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();

    @Override
    public void runOpMode() {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        waitForStart();

        while (opModeIsActive()) {
            for (LynxModule hub : hardware.allHubs) {
                hub.clearBulkCache();
            }

            robot.intake.setSpeed(IntakeSubsystem.DEFAULT_SPEED);
//            robot.shooter.doAutoShoot();

//            GamepadEx gamepad1ex = new GamepadEx(gamepad1);

            // driver 1
            robot.follower.startTeleOpDrive();
            robot.drive
                    .setDefaultCommand(new DriveCommand(
                            () -> (double) gamepad1.left_stick_x,
                            () -> (double) gamepad1.left_stick_y,
                            () -> (double) gamepad1.right_stick_x, robot)
                    );

            CommandScheduler.getInstance().run();
            hardware.write();
            robot.telemetry.addData("robot position",robot.follower.getPose());
            robot.telemetry.update();

        }

    }

}
