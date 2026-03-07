package org.firstinspires.ftc.teamcode.opmodes.auto.colored;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.opmodes.auto.Auto18;

@Config
@Autonomous(name="Blue 18", group="Leave Auto", preselectTeleOp = "🟦 Blue RC TeleOp")
public class Blue18 extends Auto18 {
    public Blue18(){
        super(Team.BLUE);
    }
}
