package org.firstinspires.ftc.teamcode.robot.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

import java.util.function.Supplier;

@Config
public class HangSubsystem extends SubsystemBase {
    public enum Position {
        FULL_90(() -> HANG_ANGLE_STOP_DEGREES, () -> HANG_SPINDEXER_POWER),
        RESTING(() -> HANG_ANGLE_TWO_DEGREES, () -> HANG_DOWN_POWER);

        public final Supplier<Double> angle;  // degrees
        public final Supplier<Double> power;

        Position(Supplier<Double> angle, Supplier<Double> power) {
            this.angle = angle;
            this.power = power;
        }
    }

    public static double HANG_SPINDEXER_POWER = 1.0; // todo: increase this if it would be good
    public static double HANG_ANGLE_STOP_DEGREES = 90.0; // todo: adjust this value

    public static double HANG_DOWN_POWER = -0.7;
    public static double HANG_ANGLE_TWO_DEGREES = 60.0;

    public static double PTO_ENGAGED_POSITION = 0.58;
    public static double PTO_DISENGAGED_POSITION = 0.35;

    private final RobotHardware hardware;

    private boolean ptoEngaged = false;
    private Position position = Position.FULL_90;

    public HangSubsystem(RobotHardware hardware) {
        this.hardware = hardware;
    }

    public boolean isPtoEngaged() {
        return ptoEngaged;
    }

    public void setPTOState(boolean state) {
        ptoEngaged = state;
    }

    public void setPosition(Position position) {
        this.position = position;
    }

    @Override
    public void periodic() {
        hardware.spindexerPTO.setPosition(ptoEngaged ? PTO_ENGAGED_POSITION : PTO_DISENGAGED_POSITION);

        if(!ptoEngaged) return;

        double pitch = hardware.imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
        Robot.debugTelemetry.addData("Robot Pitch", pitch);
        Log.i("HangSubsystem", "Robot Pitch: " + pitch + " deg");

        double power = position.power.get();
        double angle = position.angle.get();
        if (power > 0) {
            if (pitch < angle) hardware.spindexerRotate.setPower(power);
            else hardware.spindexerRotate.setPower(0);
        } else if (power < 0) {
            if (pitch > angle) hardware.spindexerRotate.setPower(power);
            else hardware.spindexerRotate.setPower(0);
        } else {
            hardware.spindexerRotate.setPower(0);
        }
    }
}
