package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Motor Test", group = "Testing")
@Config
public class MotorTest extends LinearOpMode {
    // targets
    public static String motorName = "";
    public static double motorPower = 0D;

    public static String motorName2 = "";
    public static double motorPower2 = 0D;


    @Override
    public void runOpMode() {
        waitForStart();

        while (opModeIsActive()) {
            if (!motorName.isEmpty()) {
                hardwareMap.get(DcMotor.class, motorName).setPower(motorPower);
            }

            if (!motorName2.isEmpty()) {
                hardwareMap.get(DcMotor.class, motorName2).setPower(motorPower2);
            }
        }
    }
}