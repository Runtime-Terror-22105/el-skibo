package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Team;

@TeleOp(name = "🟥 Red RC TeleOp", group = "ATeleOp")
@Config
public class RedRCTele extends TerrorTeleOp {

    public RedRCTele(){
        super(Team.RED);
    }

    @Override
    public void runOpMode() {
        super.runOpMode();
    }
}
