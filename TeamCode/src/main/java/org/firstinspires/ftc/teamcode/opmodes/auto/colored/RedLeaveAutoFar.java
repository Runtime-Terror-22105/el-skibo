package org.firstinspires.ftc.teamcode.opmodes.auto.colored;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.opmodes.auto.LeaveAutoFar;

@Autonomous(name="Red Side Leave Far Auto", group="Leave Auto")
public class RedLeaveAutoFar extends LeaveAutoFar {
    public RedLeaveAutoFar(){
        super(Team.RED);
    }
}
