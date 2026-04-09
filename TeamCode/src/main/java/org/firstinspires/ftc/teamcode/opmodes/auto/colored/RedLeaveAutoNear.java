package org.firstinspires.ftc.teamcode.opmodes.auto.colored;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.opmodes.auto.LeaveAutoNear;

@Autonomous(name="Red Leave Near Side Auto", group="Leave Auto")
public class RedLeaveAutoNear extends LeaveAutoNear {
    public RedLeaveAutoNear(){
        super(Team.RED);
    }
}
