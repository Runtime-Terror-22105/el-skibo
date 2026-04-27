package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.subsystems.HangSubsystem;

@TeleOp(name="Drivetrain Wheel PID Tuner", group="Tuning")
public class DrivetrainWheelPidTuner extends LinearOpMode {
    public static HangSubsystem.HangMotorPositionPair MOTOR_POSITIONS_DEGREES = new HangSubsystem.HangMotorPositionPair(0, 0);
    public static long LOOP_DELAY = 0;

    private final RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.AUTO);
        robot.init(hardware, this);

        waitForStart();

        while (opModeIsActive()) {
            HangSubsystem.HangMotorPositionPair currentPositions = robot.hang.getCurrentPositions();
            robot.hang.setPidTargetPositions(MOTOR_POSITIONS_DEGREES.convertFromDegreesToRadians());
            HangSubsystem.HangMotorPositionPair powers = robot.hang.calculateHangMotorPowers(false);
            hardware.motorRearLeft.setPower(powers.getLeft());
            hardware.motorRearRight.setPower(powers.getRight());

            robot.telemetry.addData("Current Position Left (deg)", Math.toDegrees(currentPositions.getLeft()));
            robot.telemetry.addData("Target Positions (deg)", MOTOR_POSITIONS_DEGREES);
            robot.telemetry.addData("Powers", powers);
            robot.telemetry.update();

            hardware.motorRearLeft.write();
            hardware.motorRearRight.write();

            sleep(LOOP_DELAY);
        }

        robot.close();
    }
}
