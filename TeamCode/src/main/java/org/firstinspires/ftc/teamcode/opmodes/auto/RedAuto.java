package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Team;

@Config
@Autonomous(name="Red Side Auto", group="Auto")
public class RedAuto extends Auto {
    public RedAuto() {
        super(Team.RED);
    }
}
