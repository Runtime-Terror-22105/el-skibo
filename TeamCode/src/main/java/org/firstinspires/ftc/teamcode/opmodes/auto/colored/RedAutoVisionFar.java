package org.firstinspires.ftc.teamcode.opmodes.auto.colored;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoVisionFar;

@Config
@Autonomous(name="🟥 Red VISION Far", group="Auto Far")
public class RedAutoVisionFar extends AutoVisionFar {
    public RedAutoVisionFar() {
        super(Team.RED);
    }
}
