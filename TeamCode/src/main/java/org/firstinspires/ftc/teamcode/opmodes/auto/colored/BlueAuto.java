package org.firstinspires.ftc.teamcode.opmodes.auto.colored;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.opmodes.auto.Auto;

@Config
@Autonomous(name="12 Sorted Blue", group="Auto Sorted 12")
public class BlueAuto extends Auto {
    public BlueAuto() {
        super(Team.BLUE);
    }
}
