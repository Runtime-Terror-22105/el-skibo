package org.firstinspires.ftc.teamcode.opmodes.auto.colored;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.opmodes.auto.NearSide21;

@Config
@Autonomous(name="Blue 21", group="Leave Auto")
public class Blue21 extends NearSide21 {
    public Blue21(){
        super(Team.BLUE);
    }
}
