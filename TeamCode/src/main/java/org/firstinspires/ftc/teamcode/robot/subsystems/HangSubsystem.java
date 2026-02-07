package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.util.Profiler;

@Config
public class HangSubsystem extends SubsystemBase {
    public static double FIRST_ANGLE = Math.PI/2;
    public static double SECOND_ANGLE = Math.toRadians(75);
    public static double SERVO_POWER = 1.0;
    public static double HOLDING_POWER = 0.1;
    public static double FINAL_TOLERANCE = Math.toRadians(5);

    public static boolean debug = true;

    private final Robot robot;

    public HangSubsystem(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void periodic() {
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
