package org.firstinspires.ftc.teamcode.opmodes.testing;

import static org.firstinspires.ftc.teamcode.robot.subsystems.ShooterSubsystem.TICKS_PER_REV;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.hardware.motors.TerrorMotorNormal;
import org.firstinspires.ftc.teamcode.robot.hardware.sensors.TerrorEncoder;

@TeleOp(name = "Motor Test", group = "Testing")
@Config
public class MotorTest extends LinearOpMode {
    // targets
    public static String motorName = "";
    public static double motorPower = 0D;

    public static String motorName2 = "";
    public static double motorPower2 = 0D;
    public static String motorName3 = "";
    public static double motorPower3 = 0D;

    public static boolean showEncoderOutput = true;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        double maxvel=0;
        while (opModeIsActive()) {
            if (!motorName.isEmpty()) {
                hardwareMap.get(DcMotor.class, motorName).setPower(motorPower);
            }

            if (!motorName2.isEmpty()) {
                hardwareMap.get(DcMotor.class, motorName2).setPower(motorPower2);
            }

            if (!motorName3.isEmpty()) {
                hardwareMap.get(DcMotor.class, motorName3).setPower(motorPower3);
            }

            if (showEncoderOutput && !motorName.isEmpty()) {
                TerrorEncoder shooterEncoder = new TerrorEncoder(new TerrorMotorNormal(hardwareMap, "motorRearLeft", 0.05, 1.0));
                shooterEncoder.setDirection(TerrorEncoder.Direction.REVERSE);
                telemetry.addData("Current velocity (ticks/sec)", shooterEncoder.getVelocity());

                // I'm pretty sure we're using this motor? https://www.gobilda.com/5202-series-yellow-jacket-planetary-gear-motor-5-2-1-ratio-1150-rpm-3-3-5v-encoder/
                telemetry.addData("Current velocity (rpm)", shooterEncoder.getVelocity() * 60 / TICKS_PER_REV); // 145.1 ticks per revolution
                maxvel=Math.max(maxvel,shooterEncoder.getVelocity() * 60 / TICKS_PER_REV);
                telemetry.addData("Max Velocity(rpm)", maxvel); // 145.1 ticks per revolution
                telemetry.addData("Current (amps)", ((DcMotorEx) hardwareMap.get(DcMotor.class, motorName)).getCurrent(CurrentUnit.AMPS));
                telemetry.update();
            }
        }
    }
}