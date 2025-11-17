package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.hardware.motors.TerrorServo;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

import java.util.Objects;

@TeleOp(name = "Servo Test", group = "Testing")
@Config
public class ServoTest extends LinearOpMode {
    // targets
    public static String servoName = "";
    public static double servoPosition = 0D;


    @Override
    public void runOpMode() {
        waitForStart();

        while (opModeIsActive()) {
            if (!servoName.isEmpty()) {
                hardwareMap.get(Servo.class, servoName).setPosition(servoPosition);
            }
        }
    }
}