package org.firstinspires.ftc.teamcode.opmodes.auto.colored;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.opmodes.auto.LeaveAutoNear;

@Config
@Autonomous(name="Red Leave Near Side Auto", group="Leave Auto")
public class RedLeaveAutoNear extends LeaveAutoNear {
    public RedLeaveAutoNear(){
        super(Team.RED);
    }
}
