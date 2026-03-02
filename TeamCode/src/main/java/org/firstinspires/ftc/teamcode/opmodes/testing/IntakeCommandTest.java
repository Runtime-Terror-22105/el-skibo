package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.LogCatCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.robot.command.DriveCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.AdjustSpindexZeroCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToIntakeStateCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToRestingStateCommand;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@Config
@TeleOp
public class IntakeCommandTest extends LinearOpMode {
    private final RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();

    public void runOpMode() {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL, RobotHardware.HardwareOptions.CAMERA);
        robot.init(hardware, this);

        waitForStart();
        GamepadEx gamepad1ex = new GamepadEx(gamepad1);

        // driver 1
        robot.follower.startTeleOpDrive();
        robot.drive
                .setDefaultCommand(new DriveCommand(
                        () -> (double) gamepad1.left_stick_x,
                        () -> (double) gamepad1.left_stick_y,
                        () -> (double) gamepad1.right_stick_x, robot)
                );

        Trigger intakeButton = new Trigger(() -> gamepad1ex.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.3);
        GamepadButton restingButton = new GamepadButton(gamepad1ex, GamepadKeys.Button.CROSS);

        GamepadButton adjustSpindexerLeft = new GamepadButton(gamepad1ex, GamepadKeys.Button.DPAD_LEFT);
        GamepadButton adjustSpindexerRight = new GamepadButton(gamepad1ex, GamepadKeys.Button.DPAD_RIGHT);

        intakeButton.whenActive(new SequentialCommandGroup(new LogCatCommand("intake testing", "button pressed"), new GoToIntakeStateCommand(robot)));
        restingButton.whenPressed(new GoToRestingStateCommand(robot));

        adjustSpindexerLeft.whileHeld(new AdjustSpindexZeroCommand(robot, false));
        adjustSpindexerRight.whileHeld(new AdjustSpindexZeroCommand(robot, true));

        while (opModeIsActive()) {
            // Manually clear the bulk read cache. Deleting this would be catastrophic b/c stale
            // vals would be used.
            for (LynxModule hub : hardware.allHubs) {
                hub.clearBulkCache();
            }

            CommandScheduler.getInstance().run();
            hardware.write();
        }

        robot.close();
    }
}
