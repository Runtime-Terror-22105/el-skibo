package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.math.controllers.PidfController;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;
import org.firstinspires.ftc.teamcode.util.Profiler;

@Config
public class HangSubsystem extends SubsystemBase {
    public static class HangMotorPositionPair {
        public double leftMotorPosition;
        public double rightMotorPosition;

        public HangMotorPositionPair(double leftMotorPosition, double rightMotorPosition) {
            this.leftMotorPosition = leftMotorPosition;
            this.rightMotorPosition = rightMotorPosition;
        }

        public double getLeft() {
            return leftMotorPosition;
        }

        public double getRight() {
            return rightMotorPosition;
        }

        public HangMotorPositionPair convertFromDegreesToRadians() {
            return new HangMotorPositionPair(
                    Math.toRadians(this.leftMotorPosition),
                    Math.toRadians(this.rightMotorPosition)
            );
        }

        @Override
        public String toString() {
            return "Left Motor Position: " + leftMotorPosition + ", Right Motor Position: " + rightMotorPosition;
        }
    }

    public static double PTO_ENGAGE_POSITION = 0;
    public static double PTO_DISENGAGE_POSITION = 0.4;
//    public static double PTO_RISE_POWER = -1;
//    public ElapsedTime hangTimer = new ElapsedTime();
//    public static double INIT_HANG_TIMER_MILLISECONDS = 250;
//    public static double HANG_TIMER_MILLISECONDS = 4000;

    // these angles are in radians and are the target angles for the rear motors to be at before engaging the PTO.
    public static HangMotorPositionPair PID_ANGLES_TO_ENGAGE = new HangMotorPositionPair(0,0);

    // This is the angle of the encoder once the robot has lifted itself up fully
    public static HangMotorPositionPair PID_ANGLES_AT_MAX_LIFT = new HangMotorPositionPair(0,0);
//    public static double LEFT_PID_ANGLE_TO_ENGAGE = 0;
//    public static double RIGHT_PID_ANGLE_TO_ENGAGE = 0;

    // PID coeffs for rear motors
    public static PidfController.PidfCoefficients LEFT_MOTOR_PID_COEFFICIENTS =
            new PidfController.PidfCoefficients(0.35, 0, 0.015, 0, 0.12);
    public static PidfController.PidfCoefficients RIGHT_MOTOR_PID_COEFFICIENTS =
            new PidfController.PidfCoefficients(0.35, 0, 0.015, 0, 0.12);

    // Pid tolerances in radians
    public static double LEFT_PID_TOLERANCE = Math.toRadians(3);
    public static double RIGHT_PID_TOLERANCE = Math.toRadians(3);

    private final Robot robot;
    private final RobotHardware hardware;

    private final PidfController leftMotorPID = new PidfController(LEFT_MOTOR_PID_COEFFICIENTS);
    private final PidfController rightMotorPID = new PidfController(RIGHT_MOTOR_PID_COEFFICIENTS);

    public HangSubsystem(RobotHardware hardware,Robot robot) {
        this.robot = robot;
        this.hardware = hardware;
        this.leftMotorPID.setTolerance(LEFT_PID_TOLERANCE);
        this.rightMotorPID.setTolerance(RIGHT_PID_TOLERANCE);
    }

    public void beginHang()
    {
        robot.robotState = RobotState.HANG_INIT;
    }

    public void setPidTargetPositions(HangMotorPositionPair targetPositions)
    {
        leftMotorPID.setTargetPosition(targetPositions.getLeft());
        rightMotorPID.setTargetPosition(targetPositions.getRight());
    }

    public HangMotorPositionPair getCurrentPositions()
    {
//        double leftMotorPosition = hardware.motorRearLeftAbsEncoder.getCurrentPosition();
//        double rightMotorPosition = hardware.motorRearRightAbsEncoder.getCurrentPosition();
        double leftMotorPosition = 0;
        double rightMotorPosition = 0;
        return new HangMotorPositionPair(leftMotorPosition, rightMotorPosition);
    }

    // Note: takeShortestPath is when we want to set the wheel to a specific angle, and take shortest path like in swerve
    // when we're actually lifting the robot, we're not trying to get to a specific angle, we want to keep going and so it doesn't make sense
    public HangMotorPositionPair calculateHangMotorPowers(boolean takeShortestPath) {
        HangMotorPositionPair currentPositions = getCurrentPositions();
        double leftMotorPower = leftMotorPID.calculatePower(currentPositions.getLeft(), 0, takeShortestPath);
        double rightMotorPower = rightMotorPID.calculatePower(currentPositions.getRight(),0, takeShortestPath);
        return new HangMotorPositionPair(leftMotorPower, rightMotorPower);
    }

    @Override
    public void periodic() {
        try (Profiler.Scope p = Profiler.enter("HangSubsystem")) {
            if(!robot.robotState.isHang())
            {
                hardware.pto.setPosition(PTO_DISENGAGE_POSITION);
                return;
            }

            // we do this to ensure the PID coefficients are up to date in case they were changed on dashboard
            leftMotorPID.setPidfCoefficients(LEFT_MOTOR_PID_COEFFICIENTS);
            rightMotorPID.setPidfCoefficients(RIGHT_MOTOR_PID_COEFFICIENTS);

            HangMotorPositionPair motorPowers;
            switch(robot.robotState) {
                // the drivetrain needs to be at the correct position before we can engage the PTO
                case HANG_INIT:
                    setPidTargetPositions(PID_ANGLES_TO_ENGAGE);
                    if (leftMotorPID.atTargetPosition() && rightMotorPID.atTargetPosition()) {
                        this.robot.robotState = RobotState.HANGING;
                    }

                    motorPowers = calculateHangMotorPowers(true);
                    hardware.motorRearRight.setPower(motorPowers.getRight());
                    hardware.motorRearLeft.setPower(motorPowers.getLeft());
                    break;

                case HANGING:
                    hardware.pto.setPosition(PTO_ENGAGE_POSITION);
                    setPidTargetPositions(PID_ANGLES_AT_MAX_LIFT);

                    motorPowers = calculateHangMotorPowers(false);
                    hardware.motorRearRight.setPower(motorPowers.getRight());
                    hardware.motorRearLeft.setPower(motorPowers.getLeft());
                    break;

                default:
                    break;
            }
        }
    }
}
