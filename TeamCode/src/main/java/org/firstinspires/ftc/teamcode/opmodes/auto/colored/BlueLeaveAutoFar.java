package org.firstinspires.ftc.teamcode.opmodes.auto.colored;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.opmodes.auto.LeaveAutoFar;

@Config
@Autonomous(name="Blue Side Leave Far Auto", group="Leave Auto")
public class BlueLeaveAutoFar extends LeaveAutoFar {
    public BlueLeaveAutoFar(){
        super(Team.BLUE);
    }
}
