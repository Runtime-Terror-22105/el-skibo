package org.firstinspires.ftc.teamcode.robot.drive.mecanum;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.math.Coordinate;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.hardware.motors.TerrorMotor;

//@Config
public class MecanumDrivetrain extends SubsystemBase implements Drivetrain {
    private static final double FREEZE_TIME = 500;

    private final TerrorMotor motorRearLeft;
    private final TerrorMotor motorFrontLeft;
    private final TerrorMotor motorRearRight;
    private final TerrorMotor motorFrontRight;

    private ElapsedTime freezeTimer;
    private boolean frozen = false;

    /**
     * Initializes a swerve drivetrain.
     * @param motorRearLeft Self explanatory
     * @param motorFrontLeft Self explanatory
     * @param motorRearRight Self explanatory
     * @param motorFrontRight Self explanatory
     */
    public MecanumDrivetrain(TerrorMotor motorRearLeft, TerrorMotor motorFrontLeft,
                             TerrorMotor motorRearRight, TerrorMotor motorFrontRight) {
        this.motorRearLeft = motorRearLeft;
        this.motorFrontLeft = motorFrontLeft;
        this.motorRearRight = motorRearRight;
        this.motorFrontRight = motorFrontRight;

        unfreeze();
    }

    /**
     * Move the robot by some amount
     * @param velocity Movement on x and y
     * @param rotation Clockwise rotation
     */
    @Override
    public void move(@NonNull Coordinate velocity, double rotation) {
        this.motorFrontLeft.setPower(velocity.y + velocity.x + rotation);
        this.motorRearLeft.setPower(velocity.y - velocity.x + rotation);
        this.motorFrontRight.setPower(velocity.y - velocity.x - rotation);
        this.motorRearRight.setPower(velocity.y + velocity.x - rotation);
        unfreeze();
    }

    public void setPower(double power) {
        this.motorFrontLeft.setPower(power);
        this.motorFrontRight.setPower(power);
        this.motorRearLeft.setPower(power);
        this.motorRearRight.setPower(power);
        unfreeze();
    }

    public void freeze() {
        this.frozen = true;
    }

    public void unfreeze() {
        this.freezeTimer = null;
        this.frozen = false;
    }
//
//    public void setPower(double frontLeftPower, double frontRightPower, double rearLeftPower, double rearRightPower) {
//        this.motorFrontLeft.setPower(frontLeftPower);
//        this.motorFrontRight.setPower(frontRightPower);
//        this.motorRearLeft.setPower(rearLeftPower);
//        this.motorRearRight.setPower(rearRightPower);
//    }
}
