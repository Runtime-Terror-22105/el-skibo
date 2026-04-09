package org.firstinspires.ftc.teamcode.opmodes.auto.colored;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.opmodes.auto.LeaveAutoNear;

@Autonomous(name="Blue Leave Near Side Auto", group="Leave Auto")
public class BlueLeaveAutoNear extends LeaveAutoNear {
    public BlueLeaveAutoNear(){
        super(Team.BLUE);
    }
}
