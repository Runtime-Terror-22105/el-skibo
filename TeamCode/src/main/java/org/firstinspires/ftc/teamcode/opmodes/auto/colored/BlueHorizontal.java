package org.firstinspires.ftc.teamcode.opmodes.auto.colored;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoHorizontal;

@Config
@Autonomous(name="🟦 Blue Horizontal", group="Auto Horizontal",preselectTeleOp = "🟦 Blue RC TeleOp")
public class BlueHorizontal extends AutoHorizontal {
    public BlueHorizontal() {
        super(Team.BLUE);
    }
}
