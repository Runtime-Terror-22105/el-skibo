package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Team;

@Config
@Autonomous(name="Red Leave Near Side Auto", group="Leave Auto")
public class RedLeaveAutoNear extends LeaveAutoNear{
    public RedLeaveAutoNear(){
        super(Team.RED);
    }
}
