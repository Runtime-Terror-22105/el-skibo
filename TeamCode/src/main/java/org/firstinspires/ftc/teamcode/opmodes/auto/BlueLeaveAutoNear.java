package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Team;

@Config
@Autonomous(name="Blue Leave Near Side Auto", group="Leave Auto")
public class BlueLeaveAutoNear extends LeaveAutoNear{
    public BlueLeaveAutoNear(){
        super(Team.BLUE);
    }
}
