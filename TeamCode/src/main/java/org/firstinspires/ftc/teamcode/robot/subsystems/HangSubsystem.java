package org.firstinspires.ftc.teamcode.robot.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
@Config
public class HangSubsystem extends SubsystemBase {
    public static double HANG_SPINDEXER_POWER = 1;
    public static double HANG_ANGLE_STOP_DEGREES = 80;
    public static double PTO_ENGAGED_POSITION = 0.8;
    public static double PTO_DISENGAGED_POSITION = 0.5;

    private final RobotHardware hardware;

    private boolean ptoEngaged = false;

    public boolean hangIsHeld;

    public HangSubsystem(RobotHardware hardware) {
        this.hardware = hardware;
        this.hangIsHeld = false;
    }

    public boolean isPtoEngaged() {
        return ptoEngaged;
    }

    public void setPTOState(boolean state) {
        ptoEngaged = state;
        hardware.spindexerPTO.setPosition(ptoEngaged ? PTO_ENGAGED_POSITION : PTO_DISENGAGED_POSITION);
    }

    @Override
    public void periodic() {
        if(!ptoEngaged) return;

//        double roll = hardware.imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);
//        Log.i("HangSubsystem", "Robot Roll: " + roll + " deg");

//        if (roll < HANG_ANGLE_STOP_DEGREES) hardware.spindexerRotate.setPower(HANG_SPINDEXER_POWER);
//        else hardware.spindexerRotate.setPower(0);

        if (hangIsHeld) {
            hardware.spindexerRotate.setPower(HANG_SPINDEXER_POWER);
        } else {
            hardware.spindexerRotate.setPower(0);
        }
    }
}
