package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Team;

@Config
@Autonomous(name="Blue Side Auto", group="Auto")
public class BlueAuto extends Auto {
    public BlueAuto() {
        super(Team.BLUE);
    }
}
