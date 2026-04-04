package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.util.Profiler;

@Config
public class HangSubsystem extends SubsystemBase {
//    public static double FIRST_ANGLE = Math.PI/2;
//    public static double SECOND_ANGLE = Math.toRadians(75);
//    public static double SERVO_POWER = 1.0;
//    public static double HOLDING_POWER = 0.1;
//    public static double FINAL_TOLERANCE = Math.toRadians(5);

    public static double PTO_ENGAGE_POSITION = 0.7;
    public static double PTO_DISENGAGE_POSITION = 0.2;

    public static double PTO_POWER = -1;

    public static boolean debug = true;

    public boolean isPTOEngaged = false;

    public ElapsedTime hangTimer = new ElapsedTime();

    public static double HANG_TIMER_MILLISECONDS = 4000;

    private final Robot robot;

    public HangSubsystem(Robot robot) {
        this.robot = robot;
        setPTOEngagement(false);
    }

    public void setPTOEngagement(boolean state)
    {
        isPTOEngaged = state;
    }
    public boolean isPTOEngaged()
    {
        return isPTOEngaged;
    }

    @Override
    public void periodic() {

        if(isPTOEngaged)
        {
            robot.hardware.pto.setPosition(PTO_ENGAGE_POSITION);
        }
        else {
            robot.hardware.pto.setPosition(PTO_DISENGAGE_POSITION);
        }
//        robot.hardware.pto.setPosition(PTO_DISENGAGE_POSITION);
        if(!robot.robotState.isHang() || !isPTOEngaged())
        {
            hangTimer.reset(); //could be cooked, lifes tough
            return;
        }
        if(hangTimer.milliseconds() > HANG_TIMER_MILLISECONDS)
        {
            robot.hardware.motorRearLeft.setPower(0);
            robot.hardware.motorRearRight.setPower(0);
        }
        else {
            robot.hardware.motorRearRight.setPower(PTO_POWER);
            robot.hardware.motorRearLeft.setPower(PTO_POWER);
        }



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
