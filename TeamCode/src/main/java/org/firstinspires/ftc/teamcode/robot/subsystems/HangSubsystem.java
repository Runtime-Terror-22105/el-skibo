package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.math.controllers.PidfController;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;
import org.firstinspires.ftc.teamcode.util.Profiler;

@Config
public class HangSubsystem extends SubsystemBase {
    public static double PTO_ENGAGE_POSITION = 0;
    public static double PTO_DISENGAGE_POSITION = 0.4;
    public static double PTO_RISE_POWER = -1;
    public boolean isPTOEngaged = false;
    public ElapsedTime hangTimer = new ElapsedTime();
    public static double INIT_HANG_TIMER_MILLISECONDS = 250;
    public static double HANG_TIMER_MILLISECONDS = 4000;
    public static double LEFT_PID_ANGLE = 0;
    public static double RIGHT_PID_ANGLE = 0;

    public static double LEFT_PID_TOLERANCE = 0;
    public static double RIGHT_PID_TOLERANCE = 0;

    private final Robot robot;
    private final RobotHardware hardware;

    public static PidfController.PidfCoefficients leftMotorPIDCoeff =
            new PidfController.PidfCoefficients(0.35, 0, 0.015, 0, 0.12);

    public static PidfController.PidfCoefficients rightMotorPIDCoeff =
            new PidfController.PidfCoefficients(0.35, 0, 0.015, 0, 0.12);

    public final PidfController leftMotorPID = new PidfController(leftMotorPIDCoeff);
    public final PidfController rightMotorPID = new PidfController(rightMotorPIDCoeff);

    public HangSubsystem(RobotHardware hardware,Robot robot) {
        this.robot = robot;
        this.hardware = hardware;
        this.leftMotorPID.setTargetPosition(LEFT_PID_ANGLE);
        this.leftMotorPID.setTolerance(LEFT_PID_TOLERANCE);
        this.rightMotorPID.setTargetPosition(RIGHT_PID_ANGLE);
        this.rightMotorPID.setTolerance(RIGHT_PID_TOLERANCE);
    }

    public void beginHang()
    {
        robot.robotState = RobotState.HANG_INIT;
    }

    @Override
    public void periodic() {

        if(!robot.robotState.isHang())
        {
            hardware.pto.setPosition(PTO_DISENGAGE_POSITION);
            hangTimer.reset(); //could be cooked, lifes tough
            return;
        }
//
        switch(robot.robotState)
        {
            case HANG_INIT:
                hardware.motorRearRight.setPower(leftMotorPID.calculatePower(hardware.motorRearLeft.getCurrentPosition(),0));
                hardware.motorRearLeft.setPower(rightMotorPID.calculatePower(hardware.motorRearRight.getCurrentPosition(),0));
                //this is proably wrong

                //i lowk dunno what rahul means by pid to a specific angle
                //are there encoders on the motors or smth
                //as in like abs encoders i know there are general encoders

                if(hangTimer.milliseconds() > INIT_HANG_TIMER_MILLISECONDS || (leftMotorPID.atTargetPosition() && rightMotorPID.atTargetPosition()) )
                {
                    this.robot.robotState =  RobotState.HANGING;
                    hangTimer.reset();
                }
                break;

            case HANGING:
                hardware.pto.setPosition(PTO_ENGAGE_POSITION);
                hardware.motorRearRight.setPower(PTO_RISE_POWER);
                hardware.motorRearLeft.setPower(PTO_RISE_POWER);
                if(hangTimer.milliseconds() > HANG_TIMER_MILLISECONDS)
                {
                    this.robot.robotState = RobotState.HANG_FINISH;
                }
                break;

            case HANG_FINISH: //probably best to not ever do this
                hardware.motorRearRight.setPower(0);
                hardware.motorRearLeft.setPower(0);
                break;

            default:
                break;
        }
//        }



        try (Profiler.Scope p = Profiler.enter("HangSubsystem")) {
//            boolean do90 = robot.robotState.equals(RobotState.HANGING_90);
//            boolean doFinal = robot.robotState.equals(RobotState.HANGING_FINAL);
//            if (do90 || doFinal) {
//                double robotPitch = robot.hardware.imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
//                Robot.debugTelemetry.addData("Robot Pitch", robotPitch);
//                Log.i("HangSubsystem", "Robot Pitch: " + robotPitch + " deg");
//
//                double goal = do90 ? FIRST_ANGLE : SECOND_ANGLE;
//                double servoPower;
//                if (Math.abs(robotPitch - Math.toDegrees(goal)) <= Math.toDegrees(FINAL_TOLERANCE)) {
//                    servoPower = HOLDING_POWER;
//                    if (debug) {
//                        Log.i("HangSubsystem", "Reached goal angle: " + Math.toDegrees(goal) + " deg");
//                    }
//                } else if (robotPitch < Math.toDegrees(goal)) {
//                    servoPower = SERVO_POWER;
//                } else {
//                    servoPower = -SERVO_POWER;
//                }
//
////                robot.hardware.hangLeft.setPower(servoPower);
////                robot.hardware.hangRight.setPower(servoPower);
//            }

        }
    }
}
