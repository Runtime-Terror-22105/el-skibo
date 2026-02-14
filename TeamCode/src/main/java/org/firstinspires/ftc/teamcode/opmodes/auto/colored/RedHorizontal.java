package org.firstinspires.ftc.teamcode.opmodes.auto.colored;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.opmodes.auto.Auto;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoHorizontal;

@Config
@Autonomous(name="🟥 Red Horizontal", group="Auto Horizontal")
public class RedHorizontal extends AutoHorizontal {
    public RedHorizontal() {
        super(Team.RED);
    }
}
