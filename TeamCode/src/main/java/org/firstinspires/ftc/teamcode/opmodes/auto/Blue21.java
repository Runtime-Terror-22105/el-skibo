package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Team;

@Config
@Autonomous(name="Blue 21", group="Leave Auto")
public class Blue21 extends NearSide21{
    public Blue21(){
        super(Team.BLUE);
    }
}
