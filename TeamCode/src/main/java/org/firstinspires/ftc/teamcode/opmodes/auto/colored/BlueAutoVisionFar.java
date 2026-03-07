package org.firstinspires.ftc.teamcode.opmodes.auto.colored;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoFar;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoVisionFar;

@Config
@Autonomous(name="🟦 Blue VISION Far", group="Auto Far")
public class BlueAutoVisionFar extends AutoVisionFar {
    public BlueAutoVisionFar() {
        super(Team.BLUE);
    }
}
