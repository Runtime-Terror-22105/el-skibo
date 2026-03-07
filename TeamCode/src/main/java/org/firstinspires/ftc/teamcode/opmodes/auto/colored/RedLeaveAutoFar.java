package org.firstinspires.ftc.teamcode.opmodes.auto.colored;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.opmodes.auto.LeaveAutoFar;

@Config
@Autonomous(name="Red Side Leave Far Auto", group="Leave Auto",preselectTeleOp = "🟥 Red RC TeleOp")
public class RedLeaveAutoFar extends LeaveAutoFar {
    public RedLeaveAutoFar(){
        super(Team.RED);
    }
}
