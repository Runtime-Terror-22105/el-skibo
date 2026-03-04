package org.firstinspires.ftc.teamcode.opmodes.auto.colored;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoSpamNoPushGate;

@Config
@Autonomous(name="🟦 Blue Spam No Push Gate", group="15 Near Spam Auto")
public class BlueSpamNoPushGate extends AutoSpamNoPushGate {
    public BlueSpamNoPushGate(){
        super(Team.BLUE);
    }
}
