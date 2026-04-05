package org.firstinspires.ftc.teamcode.opmodes.auto.colored;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoHorizontal;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoHorizontalSpam;

@Config
@Autonomous(name="🟦 Blue Horizontal Spam", group="Auto Horizontal Spam")
public class BlueHorizontalSpam extends AutoHorizontalSpam {
    public BlueHorizontalSpam() {
        super(Team.BLUE);
    }
}
