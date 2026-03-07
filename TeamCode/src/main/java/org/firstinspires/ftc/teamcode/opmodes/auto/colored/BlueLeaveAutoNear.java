package org.firstinspires.ftc.teamcode.opmodes.auto.colored;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.opmodes.auto.LeaveAutoNear;

@Config
@Autonomous(name="Blue Leave Near Side Auto", group="Leave Auto",preselectTeleOp = "🟦 Blue RC TeleOp")
public class BlueLeaveAutoNear extends LeaveAutoNear {
    public BlueLeaveAutoNear(){
        super(Team.BLUE);
    }
}
