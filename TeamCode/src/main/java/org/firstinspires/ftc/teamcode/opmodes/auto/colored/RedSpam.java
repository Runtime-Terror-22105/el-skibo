package org.firstinspires.ftc.teamcode.opmodes.auto.colored;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoSpam;

@Config
@Autonomous(name="🟥 Red Spam", group="15 Near Spam Auto",preselectTeleOp = "🟥 Red RC TeleOp")
public class RedSpam extends AutoSpam {
    public RedSpam(){
        super(Team.RED);
    }
}
