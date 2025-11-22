package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.math.Pose2d;

@TeleOp(name = "Blue RC TeleOp", group = "TeleOp")
@Config
public class BlueRCTele extends TerrorTeleOp {



    public BlueRCTele(){
        super(team.BLUE);
    }

    @Override
    public void runOpMode() {
        super.runOpMode();
    }
}
