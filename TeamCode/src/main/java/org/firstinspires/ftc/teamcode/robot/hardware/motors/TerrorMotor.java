package org.firstinspires.ftc.teamcode.robot.hardware.motors;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robot.hardware.TerrorWritingDevice;

public interface TerrorMotor extends TerrorWritingDevice {
    void setPower(double power);
    double getPower();
    double getLastPower();
    void setDirection(DcMotorSimple.Direction direction);
    DcMotorSimple.Direction getDirection();
}
