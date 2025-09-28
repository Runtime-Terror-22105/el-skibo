package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.math.Algebra;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@Config
public class TurretSubsystem extends SubsystemBase {
    // 320 deg of servo rotation = 408 deg of turret rotation
    public static double YAW_GEAR_RATIO = 408.0 / 320.0;

    // TODO: tune these once we get bot. Min/max pos should be opposite extremes of
    //  turret yaw.
    public static double YAW_LEFT_MIN_POS = 0.0;
    public static double YAW_LEFT_MAX_POS = 1.0;
    public static double YAW_RIGHT_MIN_POS = 0.0;
    public static double YAW_RIGHT_MAX_POS = 1.0;

    private final RobotHardware hardware;

    private double yaw; // radians, from -pi to +pi

    public TurretSubsystem(RobotHardware hardware) {
        this.hardware = hardware;
    }

    /**
     * @param angle radians, from -pi to +pi
     */
    public void setYaw(double angle) {
        yaw = Math.max(-Math.PI, Math.min(Math.PI, angle));
    }

    @Override
    public void periodic() {
        double servoYaw = yaw / YAW_GEAR_RATIO;
        servoYaw = Math.max(-Math.PI, Math.min(Math.PI, servoYaw));
        hardware.turretYawLeft.setPosition(Algebra.mapRange(
                servoYaw,
                -Math.PI, Math.PI,
                YAW_LEFT_MIN_POS, YAW_LEFT_MAX_POS
        ));
        hardware.turretYawRight.setPosition(Algebra.mapRange(
                servoYaw,
                -Math.PI, Math.PI,
                YAW_RIGHT_MIN_POS, YAW_RIGHT_MAX_POS
        ));
    }
}
