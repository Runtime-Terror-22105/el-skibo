package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.util.Profiler;

public class DriveSubsystem extends SubsystemBase {
    private final String TAG = "DriveSubsystem";
    private final Robot robot;
    public boolean killMotors;
    public boolean slowSpeed;
    private boolean headingLock;

    public DriveSubsystem(Robot robot) {
        this.robot = robot;
        this.killMotors = false;
        this.slowSpeed = false;
        this.headingLock = false;
    }

    public boolean isHeadingLocked() {
        return this.headingLock;
    }

    public void toggleHeadingLock() {
        this.headingLock = !headingLock;
    }

    @Override
    public void periodic() {
        if (killMotors) {
            robot.hardware.motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.hardware.motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.hardware.motorRearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.hardware.motorRearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.hardware.motorFrontLeft.setPower(0);
            robot.hardware.motorFrontRight.setPower(0);
            robot.hardware.motorRearLeft.setPower(0);
            robot.hardware.motorRearRight.setPower(0);
            robot.follower.poseTracker.update();
            return;
        }

//        if (robot.robotState.isHang()) {
//            robot.hardware.motorFrontLeft.setPower(0);
//            robot.hardware.motorFrontRight.setPower(0);
//            robot.hardware.motorRearLeft.setPower(0);
//            robot.hardware.motorRearRight.setPower(0);
//
//            Log.d(TAG, "Currently hanging! State: " + robot.robotState);
//            return;
//        }

        try (Profiler.Scope p = Profiler.enter("DriveSubsystem")) {
            robot.follower.update();
//            robot.robotZone.setPosition(robot.follower.getPose().getX(), robot.follower.getPose().getY());
//            robot.robotZone.setRotation(robot.follower.getHeading());
//            robot.telemetry.addData("isInShootZone",robot.isInTapeZone());
        }
    }

    public void toggleSlowSpeed() {
        this.slowSpeed = !this.slowSpeed;
    }
}
