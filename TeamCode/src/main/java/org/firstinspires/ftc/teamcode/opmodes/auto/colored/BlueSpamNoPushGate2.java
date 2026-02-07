package org.firstinspires.ftc.teamcode.opmodes.auto.colored;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoSpam;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoSpamNoPushGate2;

@Config
@Autonomous(name="Blue Spam No Push Gate 2", group="15 Near Spam Auto")
public class BlueSpamNoPushGate2 extends AutoSpamNoPushGate2 {
    public BlueSpamNoPushGate2(){
        super(Team.BLUE);
    }
}
