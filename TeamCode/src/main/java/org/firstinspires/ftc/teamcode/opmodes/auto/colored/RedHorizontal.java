package org.firstinspires.ftc.teamcode.opmodes.auto.colored;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoHorizontal;

@Autonomous(name="🟥 Red Horizontal", group="Auto Horizontal")
public class RedHorizontal extends AutoHorizontal {
    public RedHorizontal() {
        super(Team.RED);
    }
}
