package org.firstinspires.ftc.teamcode.opmodes.auto.colored;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoSpam;

@Autonomous(name="🟥 Red Spam", group="15 Near Spam Auto")
public class RedSpam extends AutoSpam {
    public RedSpam(){
        super(Team.RED);
    }
}
