package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Team;

@Config
@Autonomous(name="Red Side Leave Far Auto", group="Leave Auto")
public class RedLeaveAutoFar extends LeaveAutoFar{
    public RedLeaveAutoFar(){
        super(Team.RED);
    }
}
