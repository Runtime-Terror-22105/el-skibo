package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Team;

@TeleOp(name = "🟦 Blue RC TeleOp", group = "ATeleOp")
@Config
public class BlueRCTele extends TerrorTeleOp {

    public BlueRCTele(){
        super(Team.BLUE);
    }

    @Override
    public void runOpMode() {
        super.runOpMode();
    }
}
