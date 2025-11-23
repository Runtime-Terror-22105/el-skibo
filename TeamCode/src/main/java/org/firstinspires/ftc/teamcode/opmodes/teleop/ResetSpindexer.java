package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Reset Spindexer", group = "TeleOp")
public class ResetSpindexer extends LinearOpMode {
    public void runOpMode() {
        hardwareMap.dcMotor.get("motorRearLeft").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
