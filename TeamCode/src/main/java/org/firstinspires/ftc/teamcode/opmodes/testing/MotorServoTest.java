package org.firstinspires.ftc.teamcode.opmodes.testing;

import static org.firstinspires.ftc.teamcode.robot.subsystems.ShooterSubsystem.TICKS_PER_REV;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.hardware.motors.TerrorMotorNormal;
import org.firstinspires.ftc.teamcode.robot.hardware.sensors.TerrorEncoder;

@TeleOp(name = "Motor Servo Test", group = "Testing")
@Config
public class MotorServoTest extends LinearOpMode {
    // motor stuff
    public static String motorName = "";
    public static double motorPower = 0D;

    public static String motorName2 = "";
    public static double motorPower2 = 0D;

    public static String motorName3 = "";
    public static double motorPower3 = 0D;

    public static String motorName4 = "";
    public static double motorPower4 = 0D;

    public static boolean showEncoderOutput = true;



    // servo stuff
    public static String servoName = "";
    public static double servoPosition = 0D;

    public static String servoName2 = "";
    public static double servoPosition2 = 0D;

    public static boolean useExtendedPwmRange = false;


    // cr servo stuff
    public static String crServoName = "";
    public static double crServoPower = 0D;

    public static String crServoName2 = "";
    public static double crServoPower2 = 0D;


    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        double maxvel=0;
        while (opModeIsActive()) {
            // motor stuff
            if (!motorName.isEmpty()) {
                hardwareMap.get(DcMotor.class, motorName).setPower(motorPower);
            }

            if (!motorName2.isEmpty()) {
                hardwareMap.get(DcMotor.class, motorName2).setPower(motorPower2);
            }

            if (!motorName3.isEmpty()) {
                hardwareMap.get(DcMotor.class, motorName3).setPower(motorPower3);
            }

            if (!motorName4.isEmpty()) {
                hardwareMap.get(DcMotor.class, motorName4).setPower(motorPower4);
            }

            if (showEncoderOutput && !motorName.isEmpty()) {
                TerrorEncoder shooterEncoder = new TerrorEncoder(new TerrorMotorNormal(hardwareMap, "shooterLeft", 0.05, 1.0));
                telemetry.addData("Current velocity (ticks/sec)", shooterEncoder.getVelocity());

                // I'm pretty sure we're using this motor? https://www.gobilda.com/5202-series-yellow-jacket-planetary-gear-motor-5-2-1-ratio-1150-rpm-3-3-5v-encoder/
                telemetry.addData("Current velocity (rpm)", shooterEncoder.getVelocity() * 60 / TICKS_PER_REV); // 145.1 ticks per revolution
                maxvel=Math.max(maxvel,shooterEncoder.getVelocity() * 60 / TICKS_PER_REV);
                telemetry.addData("Max Velocity(rpm)", maxvel); // 145.1 ticks per revolution
                telemetry.addData("Current (amps)", ((DcMotorEx) hardwareMap.get(DcMotor.class, motorName)).getCurrent(CurrentUnit.AMPS));
                telemetry.update();
            }


            // cr servo stuff
            if (!crServoName.isEmpty()) {
                hardwareMap.get(CRServo.class, crServoName).setPower(crServoPower);
            }

            if (!crServoName2.isEmpty()) {
                hardwareMap.get(CRServo.class, crServoName2).setPower(crServoPower2);
            }


            // servo stuff
            if (!servoName.isEmpty()) {

                Servo servo = hardwareMap.get(Servo.class, servoName);

                if (useExtendedPwmRange) {
                    int portNumber = servo.getPortNumber();
                    PwmControl.PwmRange customRange = new PwmControl.PwmRange(500, 2500);  // Adjust based on servo specs

                    ServoControllerEx servoController = (ServoControllerEx) servo.getController();
                    servoController.setServoPwmRange(portNumber, customRange);
                }

                servo.setPosition(servoPosition);
            }

            if (!servoName2.isEmpty()) {
                Servo servo2 = hardwareMap.get(Servo.class, servoName2);
                if (useExtendedPwmRange) {
                    int portNumber = servo2.getPortNumber();
                    PwmControl.PwmRange customRange = new PwmControl.PwmRange(500, 2500);  // Adjust based on servo specs

                    ServoControllerEx servoController2 = (ServoControllerEx) servo2.getController();
                    servoController2.setServoPwmRange(portNumber, customRange);
                }
                servo2.setPosition(servoPosition2);
            }
        }
    }
}