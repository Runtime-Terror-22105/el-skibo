package org.firstinspires.ftc.teamcode.robot.hardware.motors;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.hardware.TerrorWritingDevice;

import java.util.Arrays;

/**
 * A wrapper class for a PhotonDcMotor with additional features like command queuing,
 * power threshold handling, and motor enabling/disabling.
 * This class implements the TerrorHardwareDevice interface.
 */
public class TerrorDummyMotorNormal extends TerrorMotorNormal {
    public TerrorDummyMotorNormal(@NonNull HardwareMap hw, String name, double powerThreshold, double powerScale) {
        super(hw, name, powerThreshold, powerScale);
    }

    @Override
    public synchronized void write() {
        // No-op
    }
}
